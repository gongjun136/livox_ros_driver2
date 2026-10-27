#!/usr/bin/env python3
"""SANY LiDAR run directory and supervised ROS launch (Linux, ROS Humble)."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import selectors
import shutil
import signal
import socket
import subprocess
import sys
import time

OWNER = "livox-run-bundle-v1"


def role_for_hostname(hostname):
    """Resolve deployed hostnames; never silently start master on an unknown host."""
    name = hostname.split(".", 1)[0].lower()
    roles = {"ubuntum": "master", "ubuntus": "slave",
             "sany-03-master": "master", "sany-03-slave": "slave"}
    if name not in roles:
        raise ValueError(f"unknown hostname {hostname!r}; specify --role master or slave")
    return roles[name]


def now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def event(run, kind, **values):
    # Each writer uses its own stream; never concurrently append to one JSONL.
    with (run / "events.jsonl").open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(dict(time=now(), event=kind, **values)) + "\n")


def worker(run, launch_file, vehicle):
    from launch import LaunchDescription, LaunchService
    from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent
    from launch.event_handlers import OnProcessExit, OnProcessStart
    from launch.events import Shutdown
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    def started(e, context):
        command = getattr(e, "cmd", [])
        for index, arg in enumerate(command[:-1]):
            if arg == "--params-file":
                source = Path(command[index + 1])
                try:
                    shutil.copy2(source, run / "config" / f"{e.pid}_{index}_{source.name}")
                except OSError as error:
                    event(run, "snapshot_error", path=str(source), error=str(error))
        event(run, "node_started", name=e.process_name, pid=e.pid)

    def exited(e, context):
        event(run, "node_exited", name=e.process_name, pid=e.pid, returncode=e.returncode)
        if e.returncode:
            return [
                EmitEvent(event=Shutdown(reason=f"{e.process_name} exited {e.returncode}"))
            ]

    service = LaunchService()
    service.include_launch_description(
        LaunchDescription(
            [
                RegisterEventHandler(OnProcessStart(on_start=started)),
                RegisterEventHandler(OnProcessExit(on_exit=exited)),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(launch_file)),
                    launch_arguments={"vehicle_config": str(vehicle)}.items(),
                ),
            ]
        )
    )
    return service.run()


def classify(returncode, events, requested_stop, check_ok):
    exits = [e for e in events if e.get("event") == "node_exited"]
    started = {e["pid"] for e in events if e.get("event") == "node_started"}
    ended = {e["pid"] for e in exits}
    if returncode or any(e["returncode"] != 0 for e in exits) or started != ended:
        return "failed"
    if not started or not check_ok:
        return "failed"
    return "stopped" if requested_stop else "completed"


def prune(root, host, keep):
    """Only our completed healthy runs on this host; never follow symlinks."""
    eligible = []
    for path in root.glob("lidar_*"):
        if path.is_symlink() or not path.is_dir():
            continue
        try:
            meta = json.loads((path / "run_metadata.json").read_text())
            if (
                meta.get("owner") == OWNER
                and meta.get("hostname") == host
                and meta.get("status") in ("stopped", "completed")
                and meta.get("ended_at")
            ):
                eligible.append(path)
        except (OSError, ValueError):
            continue
    for path in sorted(eligible)[:-keep]:
        shutil.rmtree(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=["auto", "master", "slave"], default="auto")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(os.environ.get("SANY_WS", str(Path.home() / "Sany"))) / "runs",
    )
    parser.add_argument(
        "--vehicle-config",
        type=Path,
        default=Path(os.environ.get("SANY_VEHICLE_CONFIG", "/opt/conf/vehicle_config.yaml")),
    )
    parser.add_argument("--keep-normal", type=int, default=20)
    parser.add_argument("--max-run-mb", type=int, default=512)
    parser.add_argument("--worker", nargs=3, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        return worker(*(Path(x) for x in args.worker))
    if args.role == "auto":
        try:
            args.role = role_for_hostname(socket.gethostname())
        except ValueError as error:
            parser.error(str(error))
    if args.keep_normal < 1 or args.max_run_mb < 64:
        parser.error("keep-normal >= 1 and max-run-mb >= 64 required")
    from ament_index_python.packages import get_package_share_directory
    import yaml
    import fcntl

    package = Path(get_package_share_directory("livox_ros_driver2_core"))
    vehicle = yaml.safe_load(args.vehicle_config.read_text(encoding="utf-8"))
    if not isinstance(vehicle, dict) or vehicle.get("VIN") not in (
        "SANLV77EXSH000858",
        "SANLU77E8TH000006",
    ):
        parser.error("unknown or missing vehicle VIN")
    if vehicle.get("LIDAR") not in ("MASTER", "MASTER AND SLAVE"):
        parser.error("invalid vehicle LIDAR layout")
    if args.role == "slave" and vehicle["LIDAR"] != "MASTER AND SLAVE":
        parser.error("slave entry requires MASTER AND SLAVE wiring")
    roles = ["rear"] if args.role == "slave" else ["front", "left", "right"]
    if args.role == "master" and vehicle["LIDAR"] == "MASTER":
        roles.append("rear")
    root = args.output_root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    lock = (root / f".lidar_{args.role}.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error("another bundle with this role/output root is running")
    host = socket.gethostname()
    prune(root, host, args.keep_normal)
    if shutil.disk_usage(root).free < 1024**3:
        parser.error("less than 1 GiB free; archive runs before starting")
    run = root / f"lidar_{datetime.now():%Y%m%d_%H%M%S_%f}_{args.role}"
    for directory in ("config", "logs/ros", "checks"):
        (run / directory).mkdir(parents=True, exist_ok=True)
    for directory in ("config", "launch_ROS2"):
        shutil.copytree(package / directory, run / "config" / directory)
    shutil.copy2(args.vehicle_config, run / "config/vehicle_config.yaml")
    revision = package / "driver_revision.txt"
    hashes = {
        str(p.relative_to(run)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (run / "config").rglob("*")
        if p.is_file()
    }
    meta = dict(
        owner=OWNER,
        schema_version=1,
        hostname=host,
        role=args.role,
        roles=roles,
        started_at=now(),
        status="running",
        package_share=str(package),
        driver_build_revision=revision.read_text().strip() if revision.exists() else "unknown",
        config_sha256=hashes,
        ros_environment={
            k: os.environ.get(k)
            for k in (
                "ROS_DOMAIN_ID",
                "RMW_IMPLEMENTATION",
                "ROS_LOCALHOST_ONLY",
                "FASTRTPS_DEFAULT_PROFILES_FILE",
                "CYCLONEDDS_URI",
            )
        },
    )
    write_json(run / "run_metadata.json", meta)
    latest = root / f"latest_lidar_{args.role}"
    link = root / f".latest_lidar_{args.role}_{os.getpid()}"
    link.symlink_to(run.name, target_is_directory=True)
    link.replace(latest)
    print(f"LiDAR run directory: {run}", flush=True)
    env = dict(
        os.environ,
        ROS_LOG_DIR=str(run / "logs/ros"),
        PYTHONUNBUFFERED="1",
        RCUTILS_COLORIZED_OUTPUT="0",
    )
    launch = (
        package
        / "launch_ROS2"
        / ("slave_lidar.launch.py" if args.role == "slave" else "sany_lidar.launch.py")
    )
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker",
        str(run),
        str(launch),
        str(run / "config/vehicle_config.yaml"),
    ]
    log = RotatingFileHandler(
        run / "logs/driver.log", maxBytes=10 * 1024**2, backupCount=5, encoding="utf-8"
    )
    log.setFormatter(logging.Formatter("%(message)s"))
    child = subprocess.Popen(
        command,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    stop = {"requested": False, "at": None, "signal": None}

    def request_stop(signum, frame):
        if stop["at"] is None:
            stop.update(requested=True, at=time.monotonic(), signal=signum)
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)
    checker_log = (run / "logs/checks.log").open("w")
    checker = subprocess.Popen(
        [
            sys.executable,
            str(Path(__file__).with_name("lidar_bundle_check.py")),
            "--run",
            str(run),
            "--roles",
            *roles,
        ],
        env=env,
        stdout=checker_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    selector = selectors.DefaultSelector()
    selector.register(child.stdout, selectors.EVENT_READ)
    pending = b""
    quota_at = time.monotonic()
    quota_exceeded = False
    forced = False
    check_recorded = False
    try:
        while selector.get_map() or child.poll() is None:
            if checker.poll() is not None and not check_recorded:
                check_recorded = True
                meta["check_returncode"] = checker.returncode
                meta["startup_check"] = "passed" if checker.returncode == 0 else "failed"
                write_json(run / "run_metadata.json", meta)
            for key, mask in selector.select(0.2):
                data = os.read(key.fd, 65536)
                if not data:
                    selector.unregister(key.fileobj)
                    continue
                pending += data
                lines = pending.split(b"\n")
                pending = lines.pop()
                # Bound lines from native libraries that never emit a newline.
                if len(pending) > 65536:
                    lines.append(pending)
                    pending = b""
                for line in lines:
                    message = line.decode("utf-8", errors="replace")
                    log.emit(
                        logging.LogRecord("driver", logging.INFO, "", 0, message, (), None)
                    )
                    # File is the authoritative output even when the terminal closes.
                    try:
                        print(message, flush=True)
                    except (BrokenPipeError, OSError):
                        pass
            if stop["at"] and time.monotonic() - stop["at"] > 25 and not forced:
                forced = True
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if time.monotonic() >= quota_at:
                quota_at = time.monotonic() + 5
                size = sum(p.stat().st_size for p in run.rglob("*") if p.is_file())
                if (
                    size > args.max_run_mb * 1024**2
                    or shutil.disk_usage(root).free < 256 * 1024**2
                ):
                    quota_exceeded = True
                    request_stop(signal.SIGTERM, None)
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGKILL)
        code = child.wait()
        try:
            checker.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(checker.pid, signal.SIGTERM)
            try:
                checker.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(checker.pid, signal.SIGKILL)
                checker.wait()
        checker_log.close()
        if pending:
            log.emit(
                logging.LogRecord(
                    "driver",
                    logging.INFO,
                    "",
                    0,
                    pending.decode("utf-8", errors="replace"),
                    (),
                    None,
                )
            )
        log.close()
        events = (
            [json.loads(line) for line in (run / "events.jsonl").read_text().splitlines()]
            if (run / "events.jsonl").exists()
            else []
        )
        status = classify(code, events, stop["requested"], checker.returncode == 0)
        if forced or quota_exceeded:
            status = "failed"
        if status == "failed":
            try:
                subprocess.run(
                    [
                        sys.executable,
                        str(Path(__file__).with_name("lidar_bundle_check.py")),
                        "--run",
                        str(run),
                        "--roles",
                        *roles,
                        "--state-only",
                    ],
                    env=env,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=12,
                    check=False,
                )
            except (OSError, subprocess.TimeoutExpired) as error:
                write_json(run / "checks/failure_state.json", {"error": str(error)})
        meta.update(
            ended_at=now(),
            status=status,
            launch_returncode=code,
            stop_requested=stop["requested"],
            stop_signal=stop["signal"],
            forced_kill=forced,
            quota_exceeded=quota_exceeded,
            check_returncode=checker.returncode,
            node_exits=[e for e in events if e["event"] == "node_exited"],
        )
        write_json(run / "run_metadata.json", meta)
        event(
            run,
            "run_finished",
            status=status,
            launch_returncode=code,
            check_returncode=checker.returncode,
            forced_kill=forced,
        )
    print(f'LiDAR run {meta["status"]}: {run}', flush=True)
    return 0 if meta["status"] in ("stopped", "completed") else 1


if __name__ == "__main__":
    sys.exit(main())

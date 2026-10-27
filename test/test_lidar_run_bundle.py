#!/usr/bin/env python3
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
import binascii
import struct
import zlib

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from run_lidar_bundle import classify, prune, write_json, OWNER, role_for_hostname  # noqa: E402


class BundleTests(unittest.TestCase):
    def test_hostname_roles_fail_closed(self):
        for host in ("ubuntuM", "ubuntuM.local", "sany-03-master"):
            self.assertEqual(role_for_hostname(host), "master")
        for host in ("ubuntuS", "ubuntuS.local", "sany-03-slave"):
            self.assertEqual(role_for_hostname(host), "slave")
        with self.assertRaises(ValueError):
            role_for_hostname("unknown-orin")

    def test_readonly_query_validation(self):
        from lidar_bundle_check import query

        class Socket:
            def __enter__(self):
                return self

            def __exit__(self, *args):
                pass

            def settimeout(self, value):
                pass

            def sendto(self, packet, address):
                self.packet, self.address = packet, address
                assert struct.unpack_from("<H", packet, 8)[0] == 0x0101

            def recvfrom(self, size):
                key = struct.unpack_from("<H", self.packet, 28)[0]
                value = {
                    0x8000: b"TEST_SN\0",
                    0x8006: b"\x01",
                    0x8011: struct.pack("<8I", 0x02160003, *([0] * 7)),
                }[key]
                data = struct.pack("<BHHH", 0, 1, key, len(value)) + value
                head = struct.pack(
                    "<BBHIHBB6s",
                    0xAA,
                    0,
                    24 + len(data),
                    struct.unpack_from("<I", self.packet, 4)[0],
                    0x0101,
                    1,
                    0,
                    bytes(6),
                )
                return (
                    head
                    + struct.pack("<HI", binascii.crc_hqx(head, 0xFFFF), zlib.crc32(data))
                    + data,
                    self.address,
                )

        with patch("lidar_bundle_check.socket.socket", return_value=Socket()):
            result = query("192.168.2.133")["parameters"]
        self.assertEqual(result["serial_number"]["value"], "TEST_SN")
        self.assertEqual(result["work_state"]["value"], 1)
        self.assertEqual(result["hms"]["value"], ["0x02160003"])

        class BadSocket(Socket):
            def recvfrom(self, size):
                raw, addr = super().recvfrom(size)
                return raw[:-1] + bytes([raw[-1] ^ 1]), addr

        with patch("lidar_bundle_check.socket.socket", return_value=BadSocket()):
            self.assertTrue(
                all("error" in v for v in query("192.168.2.133")["parameters"].values())
            )

    def test_node_crash_cannot_be_hidden_by_launch_success(self):
        events = [
            dict(event="node_started", pid=1),
            dict(event="node_exited", pid=1, returncode=-11),
        ]
        self.assertEqual(classify(0, events, True, True), "failed")
        events[-1]["returncode"] = 0
        self.assertEqual(classify(0, events, True, True), "stopped")
        self.assertEqual(classify(0, events[:1], True, True), "failed")
        self.assertEqual(classify(0, events, True, False), "failed")

    def test_retention_preserves_failures_running_foreign_and_links(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for name, status in [
                ("01", "completed"),
                ("02", "stopped"),
                ("03", "failed"),
                ("04", "running"),
            ]:
                path = root / ("lidar_" + name)
                path.mkdir()
                write_json(
                    path / "run_metadata.json",
                    dict(owner=OWNER, hostname="test", status=status, ended_at="now"),
                )
            (root / "lidar_foreign").mkdir()
            (root / "lidar_link").symlink_to(root / "lidar_02", target_is_directory=True)
            prune(root, "test", 1)
            self.assertFalse((root / "lidar_01").exists())
            for suffix in ["02", "03", "04", "foreign", "link"]:
                self.assertTrue((root / ("lidar_" + suffix)).exists())

    @unittest.skipUnless(
        sys.platform == "linux" and importlib.util.find_spec("launch"),
        "requires ROS launch on Linux",
    )
    def test_real_launch_supervision(self):
        # Real LaunchService and processes, synthetic device (no network or hardware).
        for crash in (False, True):
            with self.subTest(crash=crash), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                scripts = root / "scripts"
                scripts.mkdir()
                shutil.copy2(SCRIPTS / "run_lidar_bundle.py", scripts)
                (scripts / "lidar_bundle_check.py").write_text("import sys\nsys.exit(0)\n")
                package = root / "package"
                (package / "config").mkdir(parents=True)
                (package / "launch_ROS2").mkdir()
                # Only package lookup is replaced; launch supervision remains real.
                shim = root / "ament_index_python"
                shim.mkdir()
                (shim / "__init__.py").write_text("")
                (shim / "packages.py").write_text(
                    f"def get_package_share_directory(name): return {str(package)!r}\n"
                )
                child = (
                    "import os,signal,time; time.sleep(1); os.kill(os.getpid(),signal.SIGSEGV)"
                    if crash
                    else "import signal,time,sys; "
                    "signal.signal(signal.SIGINT,lambda *a:sys.exit(0)); "
                    'print("READY",flush=True); time.sleep(60)'
                )
                (package / "launch_ROS2/sany_lidar.launch.py").write_text(
                    "from launch import LaunchDescription\n"
                    "from launch.actions import ExecuteProcess\n"
                    "def generate_launch_description():\n"
                    "    return LaunchDescription([ExecuteProcess(\n"
                    f'        cmd=[{sys.executable!r}, "-c", {child!r}], output="screen")])\n'
                )
                vehicle = root / "vehicle.yaml"
                vehicle.write_text("VIN: SANLV77EXSH000858\nLIDAR: MASTER AND SLAVE\n")
                env = dict(
                    os.environ,
                    PYTHONPATH=str(root) + os.pathsep + os.environ.get("PYTHONPATH", ""),
                )
                with (root / "console.log").open("w") as output:
                    process = subprocess.Popen(
                        [
                            sys.executable,
                            str(scripts / "run_lidar_bundle.py"),
                            "--output-root",
                            str(root / "runs"),
                            "--role",
                            "master",
                            "--vehicle-config",
                            str(vehicle),
                        ],
                        env=env,
                        stdout=output,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                    try:
                        if not crash:
                            deadline = time.monotonic() + 10
                            while (
                                "READY" not in (root / "console.log").read_text()
                                and time.monotonic() < deadline
                            ):
                                time.sleep(0.1)
                            self.assertIn("READY", (root / "console.log").read_text())
                            process.send_signal(signal.SIGINT)
                        code = process.wait(timeout=20)
                    finally:
                        if process.poll() is None:
                            process.kill()
                            process.wait()
                run = (root / "runs/latest_lidar_master").resolve()
                meta = json.loads((run / "run_metadata.json").read_text())
                self.assertEqual(
                    meta["status"],
                    "failed" if crash else "stopped",
                    (root / "console.log").read_text(),
                )
                self.assertEqual(code, 1 if crash else 0)
                self.assertEqual(meta["node_exits"][0]["returncode"], -11 if crash else 0)
                self.assertTrue((run / "logs/driver.log").is_file())
                self.assertTrue((run / "config/vehicle_config.yaml").is_file())


if __name__ == "__main__":
    unittest.main()

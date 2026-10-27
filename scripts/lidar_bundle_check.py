#!/usr/bin/env python3
"""Bounded startup evidence; no permanent subscriptions or device writes."""

import argparse
import binascii
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time
import zlib
from run_lidar_bundle import write_json, now

IPS = dict(
    front="192.168.3.184", left="192.168.1.108", right="192.168.2.133", rear="192.168.4.143"
)


def query(ip):
    result = {
        "ip": ip,
        "collected_at": now(),
        "source": "livox_read_only_0x0101",
        "parameters": {},
    }
    for key, name in ((0x8000, "serial_number"), (0x8006, "work_state"), (0x8011, "hms")):
        payload = struct.pack("<HHH", 1, 0, key)
        seq = time.time_ns() & 0xFFFFFFFF
        header = struct.pack("<BBHIHBB6s", 0xAA, 0, 30, seq, 0x0101, 0, 0, bytes(6))
        packet = (
            header
            + struct.pack("<HI", binascii.crc_hqx(header, 0xFFFF), zlib.crc32(payload))
            + payload
        )
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.settimeout(0.75)
                sock.sendto(packet, (ip, 56100))
                raw, address = sock.recvfrom(2048)
            if (
                address[0] != ip
                or len(raw) < 27
                or raw[0] != 0xAA
                or raw[10] != 1
                or struct.unpack_from("<H", raw, 2)[0] != len(raw)
                or struct.unpack_from("<I", raw, 4)[0] != seq
                or struct.unpack_from("<H", raw, 8)[0] != 0x0101
                or binascii.crc_hqx(raw[:18], 0xFFFF) != struct.unpack_from("<H", raw, 18)[0]
                or zlib.crc32(raw[24:]) != struct.unpack_from("<I", raw, 20)[0]
            ):
                raise ValueError("invalid response header/CRC")
            row = {"return_code": raw[24], "raw_hex": raw.hex()}
            if raw[24] == 0:
                if len(raw) < 31 or struct.unpack_from("<HH", raw, 25) != (1, key):
                    raise ValueError("missing/mismatched key")
                length = struct.unpack_from("<H", raw, 29)[0]
                value = raw[31:]
                if len(value) != length:
                    raise ValueError("invalid value length")
                if name == "serial_number":
                    row["value"] = value.split(b"\0", 1)[0].decode("ascii")
                elif name == "work_state" and length == 1:
                    row["value"] = value[0]
                elif name == "hms" and length == 32:
                    row["value"] = [f"0x{x:08x}" for x in struct.unpack("<8I", value) if x]
                else:
                    raise ValueError("unexpected value length")
            result["parameters"][name] = row
        except (OSError, ValueError, struct.error) as error:
            result["parameters"][name] = {"error": str(error)}
    return result


def receive(roles, seconds=20):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, Imu

    rclpy.init()
    node = rclpy.create_node("lidar_bundle_startup_check")
    rows, subscriptions = {}, []
    for role in roles:
        for kind, msg_type in [("lidar", PointCloud2), ("imu", Imu)]:
            topic = "/livox/" + kind + "_" + IPS[role].replace(".", "_")
            row = rows[topic] = dict(count=0, nonmonotonic=0, empty_clouds=0)

            def callback(msg, row=row):
                tick = time.monotonic()
                stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                if row["count"] and stamp <= row["last_stamp_ns"]:
                    row["nonmonotonic"] += 1
                row.setdefault("first", tick)
                row.update(
                    last=tick, last_stamp_ns=stamp, stamp_age_sec=time.time() - stamp / 1e9
                )
                row["count"] += 1
                if hasattr(msg, "width"):
                    row["last_points"] = msg.width * msg.height
                    row["empty_clouds"] += int(row["last_points"] == 0)

            subscriptions.append(
                node.create_subscription(msg_type, topic, callback, qos_profile_sensor_data)
            )
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    for row in rows.values():
        row["arrival_age_sec"] = time.monotonic() - row["last"] if row["count"] else None
        row["hz"] = (
            (row["count"] - 1) / (row["last"] - row["first"])
            if row["count"] > 1 and row["last"] > row["first"]
            else 0
        )
        row["passed"] = (
            row["count"] >= 3
            and row["arrival_age_sec"] < 1
            and row["nonmonotonic"] == 0
            and row["empty_clouds"] == 0
            and -0.1 <= row["stamp_age_sec"] < 1
        )
    node.destroy_node()
    rclpy.shutdown()
    return dict(passed=all(r["passed"] for r in rows.values()), topics=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--roles", nargs="+", choices=list(IPS), required=True)
    parser.add_argument("--state-only", action="store_true")
    args = parser.parse_args()
    checks = args.run / "checks"
    if args.state_only:
        write_json(
            checks / "failure_state.json", {role: query(IPS[role]) for role in args.roles}
        )
        return 0
    try:
        data = receive(args.roles)
    except Exception as error:
        data = dict(passed=False, error=str(error))
    write_json(checks / "startup_check.json", data)
    try:
        result = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).with_name("collect_lidar_identity.py")),
                "--roles",
                *args.roles,
                "--timeout",
                "5",
                "--vehicle-config",
                str(args.run / "config/vehicle_config.yaml"),
                "--output",
                str(checks / "identity.json"),
            ],
            timeout=12,
            check=False,
        )
        identity_ok = result.returncode == 0
    except (OSError, subprocess.TimeoutExpired) as error:
        identity_ok = False
        write_json(checks / "identity.json", dict(complete=False, error=str(error)))
    devices = {role: query(IPS[role]) for role in args.roles}
    write_json(checks / "device_state.json", devices)
    state_ok = all(
        d["parameters"]["work_state"].get("value") == 1
        and "value" in d["parameters"]["hms"]
        and not any(int(code, 16) & 0xFF >= 3 for code in d["parameters"]["hms"]["value"])
        for d in devices.values()
    )
    write_json(
        checks / "result.json",
        dict(
            passed=data["passed"] and identity_ok and state_ok,
            data_ok=data["passed"],
            identity_ok=identity_ok,
            device_state_ok=state_ok,
        ),
    )
    return 0 if data["passed"] and identity_ok and state_ok else 2


if __name__ == "__main__":
    sys.exit(main())

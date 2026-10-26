#!/usr/bin/env python3
"""Collect last SDK-confirmed identities; this is not a device liveness check."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
from datetime import datetime, timezone

ROLES = ('front', 'left', 'right', 'rear')


def topic_for(role):
    return f'/sensors/livox/{role}/device_info'


def identity_record(role, msg):
    if msg.role != role:
        raise ValueError(f'{topic_for(role)} carried unexpected role {msg.role!r}')
    stamp = msg.header.stamp
    valid = bool(msg.identity_valid and msg.serial_number and msg.ip and
                 (stamp.sec != 0 or stamp.nanosec != 0))
    return dict(role=role, topic=topic_for(role), ip=msg.ip,
                serial_number=msg.serial_number if valid else None,
                device_type=int(msg.device_type), identity_valid=valid,
                confirmed_at_ros=dict(sec=stamp.sec, nanosec=stamp.nanosec),
                frame_id=msg.header.frame_id, source='livox_sdk',
                status='identified' if valid else 'unknown')


def make_inventory(roles, records, conflicts, vin):
    devices = []
    for role in roles:
        item = dict(records.get(role, dict(role=role, topic=topic_for(role), ip=None,
                    serial_number=None, identity_valid=False, source=None,
                    confirmed_at_ros=None, status='not_received')))
        if role in conflicts:
            item.update(identity_valid=False, serial_number=None, status='conflict')
        devices.append(item)
    return dict(schema_version=1, vin=vin,
                collected_at_utc=datetime.now(timezone.utc).isoformat(),
                semantics='Last SDK-confirmed identity; consult diagnostic_state for liveness.',
                complete=all(item['identity_valid'] for item in devices), devices=devices)


def main():
    import rclpy
    from rclpy.utilities import remove_ros_args
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from livox_ros_driver2.msg import LidarDeviceInfo
    import yaml

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--roles', nargs='+', choices=ROLES, default=list(ROLES))
    parser.add_argument('--timeout', type=float, default=5.0, help='collection window in seconds')
    parser.add_argument('--vin', help='explicit vehicle VIN (otherwise read vehicle-config)')
    parser.add_argument('--vehicle-config', type=Path, default=Path('/opt/conf/vehicle_config.yaml'))
    parser.add_argument('--output', type=Path, help='write JSON inventory to this file')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be a finite positive number')
    roles = list(dict.fromkeys(args.roles))
    vin = args.vin
    if not vin:
        try:
            config = yaml.safe_load(args.vehicle_config.read_text(encoding='utf-8'))
            vin = config.get('VIN') if isinstance(config, dict) else None
            if not isinstance(vin, str) or not vin.strip():
                vin = None
        except (OSError, yaml.YAMLError) as error:
            print(f'VIN unavailable: {error}', file=sys.stderr)
    records, conflicts = {}, set()
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('collect_lidar_identity')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def receive(role, msg):
        try:
            records[role] = identity_record(role, msg)
        except ValueError as error:
            conflicts.add(role)
            print(error, file=sys.stderr)

    subscriptions = [node.create_subscription(LidarDeviceInfo, topic_for(role),
                     lambda msg, role=role: receive(role, msg), qos) for role in roles]
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            for role in roles:
                if node.count_publishers(topic_for(role)) > 1:
                    conflicts.add(role)
        inventory = make_inventory(roles, records, conflicts, vin)
        print(f'VIN: {vin or "UNKNOWN"} (last confirmed identity, not online status)')
        print(f'{"ROLE":8} {"IP":16} {"SN":18} STATUS')
        for item in inventory['devices']:
            print(f'{item["role"]:8} {(item.get("ip") or "-"):16} '
                  f'{(item.get("serial_number") or "UNKNOWN"):18} {item["status"]}')
        if args.output:
            args.output.write_text(json.dumps(inventory, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        return 0 if inventory['complete'] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

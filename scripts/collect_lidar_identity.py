#!/usr/bin/env python3
"""Query last SDK-confirmed identities via ROS parameters, not liveness."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
from datetime import datetime, timezone

ROLES = ('front', 'left', 'right', 'rear')
FIELDS = ('role', 'ip', 'serial_number', 'device_type', 'identity_valid',
          'confirmed_at_ns', 'frame_id')


def node_for(role):
    return f'/livox_{role}_publisher'


def identity_record(role, values, node_name=None):
    if values.get('role') != role:
        raise ValueError(f'expected role {role}, received {values.get("role")!r}')
    expected = (str, str, str, int, bool, int, str)
    if any(type(values.get(key)) is not kind for key, kind in zip(FIELDS, expected)):
        raise ValueError('missing or incorrectly typed identity parameters')
    stamp = values['confirmed_at_ns']
    valid = bool(values['identity_valid'] and values['serial_number'] and values['ip'] and stamp > 0)
    return dict(role=role, node=node_name or node_for(role), ip=values['ip'],
                serial_number=values['serial_number'] if valid else None,
                device_type=values['device_type'], identity_valid=valid,
                confirmed_at_ros=dict(sec=stamp // 1000000000, nanosec=stamp % 1000000000),
                frame_id=values['frame_id'], source='livox_sdk',
                status='identified' if valid else 'unknown')


def make_inventory(roles, records, conflicts, vin, nodes=None):
    nodes = nodes or {role: node_for(role) for role in roles}
    devices = []
    for role in roles:
        item = dict(records.get(role, dict(role=role, node=nodes[role], ip=None,
                    serial_number=None, identity_valid=False, source=None,
                    confirmed_at_ros=None, status='not_received')))
        if role in conflicts:
            item.update(identity_valid=False, serial_number=None, status='conflict')
        devices.append(item)
    return dict(schema_version=2, vin=vin,
                collected_at_utc=datetime.now(timezone.utc).isoformat(),
                semantics='Last SDK-confirmed identity; consult diagnostic_state for liveness.',
                complete=all(item['identity_valid'] for item in devices), devices=devices)


def main():
    import rclpy
    from rclpy.utilities import remove_ros_args
    from rclpy.parameter import parameter_value_to_python
    from rcl_interfaces.srv import GetParameters
    import yaml

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--roles', nargs='+', choices=ROLES, default=list(ROLES))
    parser.add_argument('--timeout', type=float, default=5.0, help='discovery/query window in seconds')
    parser.add_argument('--node', action='append', default=[], metavar='ROLE=/NODE',
                        help='override a role node name, including its namespace')
    parser.add_argument('--vin', help='explicit VIN, otherwise read vehicle-config')
    parser.add_argument('--vehicle-config', type=Path, default=Path('/opt/conf/vehicle_config.yaml'))
    parser.add_argument('--output', type=Path, help='write JSON inventory to this file')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be a finite positive number')
    roles = list(dict.fromkeys(args.roles))
    nodes = {role: node_for(role) for role in roles}
    for override in args.node:
        role, sep, name = override.partition('=')
        if not sep or role not in nodes or not name.startswith('/') or name.endswith('/'):
            parser.error('--node must be ROLE=/fully/qualified/node for a requested role')
        nodes[role] = name
    if len(set(nodes.values())) != len(nodes):
        parser.error('each role must have a distinct node')
    vin = args.vin
    if not vin:
        try:
            config = yaml.safe_load(args.vehicle_config.read_text(encoding='utf-8'))
            vin = config.get('VIN') if isinstance(config, dict) else None
            if not isinstance(vin, str) or not vin.strip():
                vin = None
        except (OSError, yaml.YAMLError) as error:
            print(f'VIN unavailable: {error}', file=sys.stderr)
    records, conflicts, futures = {}, set(), {}
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('collect_lidar_identity')
    clients = {role: node.create_client(GetParameters, name + '/get_parameters')
               for role, name in nodes.items()}
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            graph = [namespace.rstrip('/') + '/' + name
                     for name, namespace in node.get_node_names_and_namespaces()]
            for role, client in clients.items():
                if graph.count(nodes[role]) > 1:
                    conflicts.add(role)
                if role not in futures and role not in conflicts and client.service_is_ready():
                    request = GetParameters.Request()
                    request.names = ['device_info.' + field for field in FIELDS]
                    futures[role] = client.call_async(request)
                future = futures.get(role)
                if future is not None and future.done() and role not in records:
                    try:
                        response = future.result()
                        if len(response.values) != len(FIELDS):
                            raise ValueError('incomplete identity response')
                        values = dict(zip(FIELDS, map(parameter_value_to_python, response.values)))
                        records[role] = identity_record(role, values, nodes[role])
                    except (ValueError, TypeError, AttributeError, RuntimeError) as error:
                        records[role] = dict(role=role, node=nodes[role], identity_valid=False,
                                             serial_number=None, source=None, status='invalid_response')
                        print(f'{nodes[role]}: {error}', file=sys.stderr)
        inventory = make_inventory(roles, records, conflicts, vin, nodes)
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

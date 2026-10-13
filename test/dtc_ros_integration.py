#!/usr/bin/env python3
"""Run the real aggregator on an isolated ROS domain; no device or SDK needed.

source install/setup.bash
ROS_DOMAIN_ID=166 python3 src/livox_ros_driver2/test/dtc_ros_integration.py
"""
import os
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_monitor_interfaces.msg import DtcObservation, DtcObservationArray
from livox_ros_driver2.msg import LidarDiagnosticState

ROLES = ('front', 'rear', 'left', 'right')
IDS = (7, 12, 8, 9)
BASES = (0x110476C4, 0x11037660, 0x11017598, 0x110275FC)
OFFSETS = (0, 1, 3, 4, 7, 8, 9, 10, 11, 12, 13)


def main():
    os.environ.setdefault('ROS_DOMAIN_ID', '166')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    node = rclpy.create_node('dtc_integration_probe')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    messages = []
    node.create_subscription(DtcObservationArray, '/diagnostics/source_dtc/livox',
                             lambda msg: messages.append(msg), qos)
    pubs = [node.create_publisher(LidarDiagnosticState, f'/sensors/livox/{role}/diagnostic_state', qos)
            for role in ROLES]
    from ament_index_python.packages import get_package_prefix
    executable = os.path.join(get_package_prefix('livox_ros_driver2_core'),
                              'lib', 'livox_ros_driver2_core', 'lidar_dtc_aggregator')
    process = subprocess.Popen([
        executable, '--ros-args', '-p', 'startup_grace_ms:=600', '-p', 'state_timeout_ms:=300',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    seq = [0] * 4

    def transmit(usability=(1, 1, 1, 1), hms_age=0, repeat=False):
        for role, pub in enumerate(pubs):
            if not repeat:
                seq[role] += 1
            msg = LidarDiagnosticState()
            msg.role = role
            msg.comm_header.source_id = IDS[role]
            msg.comm_header.seq = seq[role]
            msg.boot_id = role + 1
            msg.config_version = 'livox-dtc-v1-bench'
            msg.usability = usability[role]
            for offset in OFFSETS:
                obs = DtcObservation()
                obs.dtc = BASES[role] + offset
                obs.physical_condition = 1
                obs.evaluation_seq = seq[role]
                obs.evaluation_age_us = hms_age
                msg.observations.append(obs)
            pub.publish(msg)

    def run(duration, states=None, hms_age=0, repeat=False):
        end = time.monotonic() + duration
        next_send = 0.0
        while time.monotonic() < end:
            if states is not None and time.monotonic() >= next_send:
                transmit(states, hms_age, repeat)
                next_send = time.monotonic() + 0.05
            rclpy.spin_once(node, timeout_sec=0.01)
            if process.poll() is not None:
                raise RuntimeError(process.stderr.read().decode())

    def latest():
        assert messages, ('aggregator did not publish; nodes=' + repr(node.get_node_names()) +
                          '; matched_driver_subscriptions=' + repr([pub.get_subscription_count() for pub in pubs]) +
                          '; dtc_publishers=' + repr(node.count_publishers('/diagnostics/source_dtc/livox')))
        msg = messages[-1]
        assert len(msg.observations) == 46
        assert len(set(obs.dtc for obs in msg.observations)) == 46
        assert [obs.dtc for obs in msg.observations] == sorted(obs.dtc for obs in msg.observations)
        assert msg.comm_header.source_id == 13 and msg.boot_id != 0 and msg.interface_version == 1
        return {obs.dtc: obs.physical_condition for obs in msg.observations}

    try:
        run(4.0, (1, 1, 1, 1))
        assert set(latest().values()) == {1}, 'all healthy snapshot'
        run(0.35, (2, 2, 1, 1))
        assert latest()[0x10F00001] == 2 and latest()[0x10F00002] == 1, 'two unavailable'
        run(0.35, (2, 2, 2, 2))
        assert latest()[0x10F00001] == 2 and latest()[0x10F00002] == 2, 'all unavailable activates both'
        run(0.35, (1, 1, 1, 1))
        assert set(latest().values()) == {1}, 'fresh recovery'
        run(0.5, (1, 1, 1, 1), repeat=True)
        assert latest()[0x10F00002] == 2, 'replayed sequence must not refresh liveness'
        run(0.35, (1, 1, 1, 1), hms_age=3_000_000)
        result = latest()
        assert result[BASES[0]] == 0 and result[0x10F00002] == 1, 'HMS expiry independent of usability'
        run(0.35, (1, 1, 1, 1))
        run(0.5)
        result = latest()
        assert result[BASES[0]] == 0 and result[0x10F00002] == 2, 'source outage keeps HMS unknown and marks unavailable'
        assert all(messages[i].comm_header.seq != messages[i - 1].comm_header.seq for i in range(1, len(messages)))
        print('dtc_ros_integration passed: complete snapshots, two/four failures, recovery, replay, HMS expiry, outage')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        stderr = process.stderr.read().decode(errors='replace')
        if stderr:
            print(stderr)


if __name__ == '__main__':
    main()

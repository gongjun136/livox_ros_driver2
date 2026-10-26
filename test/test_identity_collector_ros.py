"""Exercise the real collector process against durable synthetic ROS publishers."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from livox_ros_driver2.msg import LidarDeviceInfo

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'collect_lidar_identity.py'


class CollectorRosTest(unittest.TestCase):
    def test_durable_delivery_missing_role_and_duplicate_publishers(self):
        rclpy.init()
        node = rclpy.create_node('synthetic_lidar_identity')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(LidarDeviceInfo, '/sensors/livox/front/device_info', qos)
        message = LidarDeviceInfo()
        message.role, message.ip = 'front', '192.0.2.1'
        message.serial_number, message.identity_valid = 'SYNTHETIC_SN', True
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)  # Publish before the collector exists.
        try:
            with tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / 'inventory.json'

                def collect(roles):
                    process = subprocess.Popen([sys.executable, str(SCRIPT), '--roles', *roles,
                                                '--vin', 'SYNTHETIC_VIN', '--timeout', '5',
                                                '--output', str(output)], stdout=subprocess.PIPE,
                                               stderr=subprocess.PIPE, text=True)
                    deadline = time.monotonic() + 12
                    while process.poll() is None and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    if process.poll() is None:
                        process.kill()
                        process.communicate()
                        self.fail('collector did not finish')
                    stdout, stderr = process.communicate()
                    self.assertIn(process.returncode, (0, 2), stderr)
                    return process.returncode, json.loads(output.read_text())

                code, inventory = collect(['front'])
                self.assertEqual(code, 0, inventory)
                self.assertEqual(inventory['devices'][0]['serial_number'], 'SYNTHETIC_SN')
                self.assertEqual(inventory['devices'][0]['confirmed_at_ros']['sec'], message.header.stamp.sec)
                code, inventory = collect(['front', 'rear'])
                self.assertEqual(code, 2)
                self.assertEqual(inventory['devices'][1]['status'], 'not_received')
                duplicate = node.create_publisher(LidarDeviceInfo, '/sensors/livox/front/device_info', qos)
                duplicate.publish(message)
                code, inventory = collect(['front'])
                self.assertEqual(code, 2)
                self.assertEqual(inventory['devices'][0]['status'], 'conflict')
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()

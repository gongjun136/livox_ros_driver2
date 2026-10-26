"""Exercise real SDK-backed parameter services and the collector in separate processes."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'collect_lidar_identity.py'
parser = argparse.ArgumentParser()
parser.add_argument('--fixture', required=True)
ARGS, remaining = parser.parse_known_args()
sys.argv = [sys.argv[0], *remaining]


class CollectorRosTest(unittest.TestCase):
    def test_remote_query_write_rejection_missing_and_duplicate_nodes(self):
        fixtures = []
        rclpy.init()
        node = rclpy.create_node('identity_test_client')

        def start():
            process = subprocess.Popen([ARGS.fixture, '--serve'], stdout=subprocess.DEVNULL)
            fixtures.append(process)
            return process

        start()
        try:
            setter = node.create_client(SetParameters, '/livox_front_publisher/set_parameters')
            self.assertTrue(setter.wait_for_service(timeout_sec=8), 'fixture service not discovered')
            request = SetParameters.Request()
            request.parameters = [Parameter('device_info.serial_number', value='FORGED').to_parameter_msg()]
            future = setter.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            self.assertTrue(future.done(), 'parameter service timed out')
            self.assertFalse(future.result().results[0].successful, 'remote SN overwrite accepted')
            with tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / 'inventory.json'

                def collect(roles):
                    result = subprocess.run([sys.executable, str(SCRIPT), '--roles', *roles,
                                             '--vin', 'SYNTHETIC_VIN', '--timeout', '3',
                                             '--output', str(output)], capture_output=True,
                                            text=True, timeout=12)
                    self.assertIn(result.returncode, (0, 2), result.stderr)
                    return result.returncode, json.loads(output.read_text())

                code, inventory = collect(['front'])
                self.assertEqual(code, 0, inventory)
                self.assertEqual(inventory['devices'][0]['serial_number'], '1234567890ABCDEF')
                self.assertEqual(inventory['devices'][0]['node'], '/livox_front_publisher')
                self.assertGreater(inventory['devices'][0]['confirmed_at_ros']['sec'], 0)
                self.assertFalse(any(name.endswith('/device_info') for name, _ in node.get_topic_names_and_types()))
                code, inventory = collect(['front', 'rear'])
                self.assertEqual(code, 2)
                self.assertEqual(inventory['devices'][1]['status'], 'not_received')
                start()
                code, inventory = collect(['front'])
                self.assertEqual(code, 2)
                self.assertEqual(inventory['devices'][0]['status'], 'conflict')
        finally:
            for process in fixtures:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()

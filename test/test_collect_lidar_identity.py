import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / 'scripts' / 'collect_lidar_identity.py'
spec = importlib.util.spec_from_file_location('identity', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class InventoryTest(unittest.TestCase):
    def message(self, **changes):
        values = dict(role='front', ip='192.168.3.184', serial_number='TEST_SN',
                      device_type=9, identity_valid=True,
                      confirmed_at_ns=100000000020, frame_id='livox_frame')
        values.update(changes)
        return values

    def test_partial_and_conflicting_inventory(self):
        record = module.identity_record('front', self.message())
        result = module.make_inventory(['front', 'rear'], {'front': record}, set(), 'TEST_VIN')
        self.assertFalse(result['complete'])
        self.assertEqual(result['devices'][0]['serial_number'], 'TEST_SN')
        self.assertIsNone(result['devices'][1]['serial_number'])
        self.assertEqual(result['devices'][1]['status'], 'not_received')
        conflict = module.make_inventory(['front'], {'front': record}, {'front'}, None)
        self.assertFalse(conflict['complete'])
        self.assertIsNone(conflict['devices'][0]['serial_number'])
        self.assertEqual(record['serial_number'], 'TEST_SN')  # Do not mutate cached records.

    def test_invalid_identity_and_wrong_role(self):
        for msg in (self.message(identity_valid=False), self.message(serial_number=''),
                    self.message(confirmed_at_ns=0)):
            record = module.identity_record('front', msg)
            self.assertFalse(record['identity_valid'])
            self.assertIsNone(record['serial_number'])
        with self.assertRaises(ValueError):
            module.identity_record('rear', self.message())

    def test_complete_inventory_preserves_source_time(self):
        record = module.identity_record('front', self.message())
        result = module.make_inventory(['front'], {'front': record}, set(), 'TEST_VIN')
        self.assertTrue(result['complete'])
        self.assertEqual(result['devices'][0]['confirmed_at_ros'], {'sec': 100, 'nanosec': 20})
        self.assertEqual(result['devices'][0]['source'], 'livox_sdk')


if __name__ == '__main__':
    unittest.main()

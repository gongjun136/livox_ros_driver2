#!/usr/bin/env python3
"""Check SANY VIN/layout dispatch without executing any LiDAR launch action.

Run in a shell that has sourced ROS Humble and the driver workspace.
"""
import importlib.util
from pathlib import Path
import tempfile
import unittest

from launch import LaunchContext
import yaml


REPO = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("sany_lidar_launch", REPO / "launch_ROS2/sany_lidar.launch.py")
ENTRY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ENTRY)


class SanyLidarLaunchTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.vehicle = Path(temporary.name) / "vehicle_config.yaml"
        self.context = LaunchContext()
        self.context.launch_configurations["vehicle_config"] = str(self.vehicle)

    def select(self, vin, layout, device_code=None):
        self.vehicle.write_text(yaml.safe_dump({"VIN": vin, "LIDAR": layout, "DEVICE_CODE": device_code}))
        return ENTRY.launch_for_vehicle(self.context)

    def test_real_new_vehicle_vin_selects_four_master_lidars(self):
        actions = self.select("SANLU77E8TH000006", "MASTER", "SW956ELCG0858")
        self.assertEqual(len(actions), 1)
        actions[0].launch_description_source.get_launch_description(self.context)
        self.assertEqual(Path(actions[0].launch_description_source.location).name, "master_4lidar.launch.py")

    def test_old_vehicle_still_selects_three_master_lidars(self):
        actions = self.select("SANLV77EXSH000858", "MASTER AND SLAVE")
        actions[0].launch_description_source.get_launch_description(self.context)
        self.assertEqual(Path(actions[0].launch_description_source.location).name, "master_lidar.launch.py")

    def test_unregistered_identifier_cannot_replace_vin(self):
        for vin in ("UNKNOWN", "SW956ELCG0858"):
            with self.subTest(vin=vin), self.assertRaisesRegex(RuntimeError, "unknown VIN"):
                self.select(vin, "MASTER")

    def test_missing_vehicle_file_and_illegal_layout_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "SANY vehicle configuration rejected"):
            ENTRY.launch_for_vehicle(self.context)
        with self.assertRaisesRegex(RuntimeError, "LIDAR must be"):
            self.select("SANLU77E8TH000006", "SLAVE")


if __name__ == "__main__":
    unittest.main()

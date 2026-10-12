"""Select the SANY master LiDAR layout from the vehicle configuration."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def launch_for_vehicle(context):
    path = Path(LaunchConfiguration('vehicle_config').perform(context))
    try:
        vehicle = yaml.safe_load(path.read_text(encoding='utf-8'))
        if not isinstance(vehicle, dict):
            raise ValueError('expected a YAML mapping')
        vin = vehicle.get('VIN')
        if vin not in ('SANLV77EXSH000858', 'SW956ELCG0858'):
            raise ValueError(f'unknown VIN: {vin!r}')
        layouts = {'MASTER AND SLAVE': 'master_lidar.launch.py',
                   'MASTER': 'master_4lidar.launch.py'}
        if vehicle.get('LIDAR') not in layouts:
            raise ValueError("LIDAR must be 'MASTER AND SLAVE' or 'MASTER'")
    except (OSError, ValueError, TypeError, yaml.YAMLError) as error:
        raise RuntimeError(f'SANY vehicle configuration rejected ({path}): {error}') from error
    package = Path(get_package_share_directory('livox_ros_driver2_core'))
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(package / 'launch_ROS2' / layouts[vehicle['LIDAR']])))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_config', default_value='/opt/conf/vehicle_config.yaml',
                              description='Vehicle VIN and LiDAR wiring configuration'),
        OpaqueFunction(function=launch_for_vehicle),
    ])

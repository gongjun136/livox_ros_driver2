from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('livox_ros_driver2_core')) / 'config' / 'dtc_aggregator.yaml'
    return LaunchDescription([Node(
        package='livox_ros_driver2_core', executable='lidar_dtc_aggregator',
        name='lidar_dtc_aggregator', parameters=[str(config)], output='screen')])

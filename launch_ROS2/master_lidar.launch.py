import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('livox_ros_driver2')
    launch_dir = os.path.join(pkg_dir, 'launch_ROS2')

    front = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'lidar_front.launch.py')
        )
    )

    left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'lidar_left.launch.py')
        )
    )

    right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'lidar_right.launch.py')
        )
    )

    return LaunchDescription([
        front,
        left,
        right,
    ])

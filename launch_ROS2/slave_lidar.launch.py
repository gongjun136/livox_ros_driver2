import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('livox_ros_driver2')
    launch_dir = os.path.join(pkg_dir, 'launch_ROS2')

    rear = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'lidar_rear.launch.py')
        )
    )


    return LaunchDescription([
        rear
    ])

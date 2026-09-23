import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_share_dir = get_package_share_directory(
        'livox_ros_driver2_core'
    )

    default_config_path = os.path.join(
        package_share_dir,
        'config',
        'MID360s_config.json'
    )

    # ================= Launch 参数 =================

    config_file = LaunchConfiguration('config_file')
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    frame_id = LaunchConfiguration('frame_id')
    node_name = LaunchConfiguration('node_name')

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Livox JSON config file absolute path'
    )

    declare_lidar_topic = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/lidar',
        description='Output LiDAR point cloud topic'
    )

    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic',
        default_value='/livox/imu',
        description='Output IMU topic'
    )

    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='livox_frame',
        description='LiDAR message frame_id'
    )

    declare_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='livox_lidar_publisher',
        description='Livox driver node name'
    )

    # ================= Livox 驱动参数 =================
    # 参数格式与官方 msg_MID360s_launch.py 保持一致：每个参数一个独立 dict

    livox_ros2_params = [
        {'xfer_format': 0},
        {'multi_topic': 0},
        {'data_src': 0},
        {'publish_freq': 10.0},
        {'output_data_type': 0},
        {'frame_id': frame_id},
        {'lvx_file_path': '/home/livox/livox_test.lvx'},
        {'user_config_path': config_file},
        {'cmdline_input_bd_code': 'livox0000000001'},
    ]

    livox_driver = Node(
        package='livox_ros_driver2_core',
        executable='livox_ros_driver2_node',
        name=node_name,
        output='screen',
        parameters=livox_ros2_params,

        # 将驱动默认话题修改成指定话题
        remappings=[
            ('livox/lidar', lidar_topic),
            ('livox/imu', imu_topic),
        ]
    )

    return LaunchDescription([
        declare_config_file,
        declare_lidar_topic,
        declare_imu_topic,
        declare_frame_id,
        declare_node_name,
        livox_driver,
    ])

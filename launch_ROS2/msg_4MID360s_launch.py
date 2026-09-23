import os

from launch import LaunchDescription
from launch_ros.actions import Node


################### user configure parameters for ros2 start ###################
xfer_format = 1      # 0-PointCloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic = 0      # Each process only connects one LiDAR; namespace separates topics
data_src = 0         # 0-lidar, others-Invalid data src
publish_freq = 10.0  # frequency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type = 0
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.dirname(os.path.realpath(__file__))
config_path = os.path.join(cur_path, '..', 'config')
config_files = [
    'MID360s_config1.json',
    'MID360s_config2.json',
    'MID360s_config3.json',
    'MID360s_config4.json',
]
################### user configure parameters for ros2 end #####################


def generate_launch_description():
    livox_drivers = []

    for index, config_file in enumerate(config_files, start=1):
        livox_drivers.append(
            Node(
                package='livox_ros_driver2_core',
                executable='livox_ros_driver2_node',
                namespace=f'mid360_{index}',
                name=f'livox_lidar_publisher_{index}',
                output='screen',
                parameters=[
                    {'xfer_format': xfer_format},
                    {'multi_topic': multi_topic},
                    {'data_src': data_src},
                    {'publish_freq': publish_freq},
                    {'output_data_type': output_type},
                    {'frame_id': f'livox_frame_{index}'},
                    {'lvx_file_path': lvx_file_path},
                    {
                        'user_config_path': os.path.join(
                            config_path, config_file
                        )
                    },
                    {'cmdline_input_bd_code': cmdline_bd_code},
                ],
            )
        )

    return LaunchDescription(livox_drivers)

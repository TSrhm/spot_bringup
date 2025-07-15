import os
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

schedule = 1

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node', #output='screen',
					parameters=[{'use_mag': False,
					             "world_frame": "imu_sensor_frame",
					             "publish_tf": False
					             }],
					remappings=[('/imu/data_raw', '/alphasense/imu')]
		))

    return ld

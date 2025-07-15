import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (bag) time'
        ),

        # Static Transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.4','0','0','0','0','0','spot/base_link','os_sensor'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.3','0','0','1.5708', '0', '3.1416','spot/base_link','base_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # RTAB-Map Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'spot/body',
                'visual_odometry': False,
                'odom_frame_id': 'spot/odom',
                'map_always_update': False,
                'wait_for_transform': 0.5,
                'subscribe_rgbd': False,
                'subscribe_rgb': False,
                'subscribe_depth': False,
                'approx_sync': True,
                'wait_imu_to_init': False,
                'subscribe_scan_cloud': True,
                'topic_queue_size': 100,
                'approx_sync': True,
                'queue_size': 100
    
            }],
            remappings=[
                ('scan_cloud', '/ouster/points'),
                ('imu', '/imu/data'),
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', '/home/imech/ros2_ws/src/spot_bringup/config/rtabmap.rviz']
        ),

        # Bag Play (mit --clock)
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                '/home/imech/ros2_logs/bags/exploration_bag/SLAM/rosbag2_2025_07_12-22_22_15/rosbag2_2025_07_12-22_22_15_0.db3',
                '--clock'
            ],
            output='screen'
        )
    ])


import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # Launch Argument für use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Static Transform Publisher Nodes
    tf1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['-0.4', '0', '0', '0', '0', '0', 'spot/base_link', 'os_sensor'],
        parameters=[{'use_sim_time': True}]
    )

    tf2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['0.3', '0', '0', '0', '0', '3.14159', 'spot/base_link', 'base_link'],
        parameters=[{'use_sim_time': True}]
    )

    # Pfad zur SLAM-Konfiguration und -Launchdatei
    slam_config = os.path.join(
        get_package_share_directory("spot_bringup"),
        "config",
        "lidarslam.yaml"
    )

    slam_launch = os.path.join(
        get_package_share_directory("lidar_localization_ros2"),
        "launch",
        "lidar_localization.launch.py"
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            "slam_params_file": slam_config,
            "use_sim_time": use_sim_time
        }.items()
    )

    # Bag record
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--all'],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/imech/ros2_ws/src/spot_bringup/config/lidarslam.rviz'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Bag playback (mit --clock, damit /clock gesendet wird)
    bag_play = TimerAction(
    period=0.0,  # Wartezeit in Sekunden
    actions=[
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                '/home/imech/ros2_logs/bags/exploration_bag/SLAM/rosbag2_2025_07_12-22_22_15/rosbag2_2025_07_12-22_22_15_0.db3',
                '--clock'
            ],
            output='screen'
        )
    ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        tf1,
        tf2,
        slam,
        bag_record,
        rviz,
        bag_play
    ])


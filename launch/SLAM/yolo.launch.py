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
        arguments=['0.3', '0', '0',  '1.5708', '0', '3.1416', 'spot/base_link', 'base_link'],
        parameters=[{'use_sim_time': True}]
    )
    #1.5708 0.7854

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--all'],
        output='screen'
    )
    
    yolo = os.path.join(get_package_share_directory("yolo_bringup"), 'launch', "yolo.launch.py")
   
   
    yolo_launch =IncludeLaunchDescription(PythonLaunchDescriptionSource(yolo),)
    
    slam_config = "/home/imech/ros2_ws/src/spot_bringup/config/slam_params1.yaml"
    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"), 'launch', "online_async_launch.py")
    slam = IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch),launch_arguments={"slam_params_file": slam_config}.items())

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/imech/ros2_ws/src/spot_bringup/config/yolo.rviz'],
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
        bag_record,
        rviz,
        bag_play,
        yolo_launch,
        slam
    ])


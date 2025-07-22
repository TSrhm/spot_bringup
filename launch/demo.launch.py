import os
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from datetime import datetime


def generate_launch_description():
    ld = LaunchDescription()

    tf = Node(package="tf2_ros", executable="static_transform_publisher",arguments=['-0.4','0','0.0','0','0','0','base_link','os_sensor'])
    ld.add_action(tf) 
    
    spot_config = "/home/imech/ros2_ws/src/spot_bringup/config/spot_ros_example.yaml"
    spot_launch = 			os.path.join(get_package_share_directory('spot_driver'),'launch',"spot_driver.launch.py")
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(spot_launch),launch_arguments={"config_file": spot_config}.items()))
    
    os_launch = os.path.join(get_package_share_directory("ouster_ros"), 'launch', "driver.launch.py")
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(os_launch),))

    cmd_node = Node(package="scan_help", executable="cmd_vel_relay",)
    ld.add_action(cmd_node)
    
    slam_config = "/home/imech/ros2_ws/src/spot_bringup/config/slam_params.yaml"
    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"), 'launch', "online_async_launch.py")
    slam_start = TimerAction(
    period=15.0,  # Sekunden Verzögerung
    actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={"slam_params_file": slam_config}.items()
        )
    ]
)
    ld.add_action(slam_start)
    
    
    nav2_launch = os.path.join(get_package_share_directory("nav2_bringup"), 'launch', "navigation_launch.py")
    #ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_launch)))

    explore_config = "/home/imech/ros2_ws/src/spot_bringup/config/explore_params.yaml"
    explore = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        parameters=[explore_config],
    )
    ld.add_action(explore)
    bag_name = f"bag_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    bag_path = f"/home/imech/ros2_ws/src/spot_bringup/bags/{bag_name}"
    bag_record = ExecuteProcess(cmd=['ros2', 'bag', 'record','-o', bag_path,'/ouster/scan', '/ouster/points', '/map', '/tf'],output='screen')
    
    ld.add_action(bag_record)

    return ld


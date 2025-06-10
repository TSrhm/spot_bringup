import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    tf = Node(package="tf2_ros", executable="static_transform_publisher",arguments=['0','0','0','0','0','0','spot','camera'])
    #ld.add_action(tf) 
    #<exec_depend> </exec_depend>

    nav2_config = "/home/imech/ros2_ws/src/spot_bringup/config/nav2_params.yaml"
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"), 'launch', "navigation_launch.py")
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_launch),launch_arguments={"params_file":nav2_config}.items()))

    config = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params.yaml"
    )
    explore = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        parameters=[config],
    )
    ld.add_action(explore)

    #os_launch = os.path.join(
    #   get_package_share_directory("ouster_ros"), 'launch', "driver.launch.py")
    #ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(os_launch),))

    cmd_node = Node(
        package="scan_help",
        executable="cmd_vel_relay",
    )
    ld.add_action(cmd_node)
    
    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"), 'launch', "online_async_launch.py")
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch)))
    
    spot_config = "/home/imech/ros2_ws/src/spot_bringup/config/spot_ros_example.yaml"
    spot_launch = os.path.join(get_package_share_directory('spot_driver'),'launch',"spot_driver.launch.py")
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(spot_launch),launch_arguments={"config_file": spot_config}.items()))

    return ld
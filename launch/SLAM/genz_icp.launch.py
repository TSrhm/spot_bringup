import os
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

schedule = 1

def generate_launch_description():
    ld = LaunchDescription()

    tf = Node(package="tf2_ros", executable="static_transform_publisher",arguments=['-0.4','0','0','0','0','0','spot/base_link','os_sensor'])
    ld.add_action(tf) 
    
    tf2 = Node(package="tf2_ros", executable="static_transform_publisher",arguments=['0.3','0','0','0','0','3.14159','spot/base_link','base_link'])
    ld.add_action(tf2) 
    
    

    
    slam_config = "/home/imech/ros2_ws/src/spot_bringup/config/genz_icp.yaml"
    slam_launch = os.path.join(
        get_package_share_directory("genz_icp"), 'launch', "odometry.launch.py")
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch),launch_arguments={"slam_params_file": slam_config,"topic": "/ouster/points","deskew": "True"}.items()))
    

    octomap = os.path.join(
        get_package_share_directory("octomap_server2"), 'launch', "octomap_server_launch.py")
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(octomap),launch_arguments={"input_cloud_topic": "/genz/local_map","frame_id": "odom", "base_frame_id": "os_lidar"}.items()))
    
  
    ld.add_action(actions.ExecuteProcess( cmd=['ros2', 'bag', 'record', "--all"],output='screen' ))

    
    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', '/home/imech/ros2_ws/src/spot_bringup/config/genz_icp.rviz'])
    
    bag_play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/imech/ros2_logs/bags/exploration_bag/SLAM/rosbag2_2025_07_12-22_22_15/rosbag2_2025_07_12-22_22_15_0.db3'],
            output='screen')
            
        
    
    ld.add_action(rviz_node)
    ld.add_action(bag_play)

    return ld

import os
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

schedule = 1

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value = "True", description='to change when using simulations or bag files'))

    tf = Node(package="tf2_ros", executable="static_transform_publisher",arguments=['-0.4','0','0','0','0','0','spot/base_link','os_sensor'])
    ld.add_action(tf) 
    
    tf2 = Node(package="tf2_ros", executable="static_transform_publisher",arguments=['0.3','0','0','0','0','3.14159','spot/base_link','base_link'])
    ld.add_action(tf2) 
    
    relay = Node(package="topic_tools", executable="relay",arguments=['/spot/odometry','spot/imu_data'])
    #ld.add_action(relay) 
    

   
    slam_launch = os.path.join(
        get_package_share_directory("rtabmap_examples"), 'launch', "lidar3d.launch.py")
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch),launch_arguments={"lidar_topic": "/ouster/points", 
     'imu_topic': '/imu/data',
     "frame_id": "imu/link",
     'scan_cloud': "ouster/points",

     }.items()))
    
    #'imu_topic': '/imu/data',
    

    
    ld.add_action(actions.ExecuteProcess( cmd=['ros2', 'bag', 'record', "--all"],output='screen' ))

    
    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', '/home/imech/ros2_ws/src/spot_bringup/config/rtabmap.rviz'])
    
    bag_play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/imech/ros2_logs/bags/exploration_bag/SLAM/rosbag2_2025_07_12-22_22_15/rosbag2_2025_07_12-22_22_15_0.db3'],
            output='screen')
            
        
    
    ld.add_action(rviz_node)
    ld.add_action(bag_play)

    return ld

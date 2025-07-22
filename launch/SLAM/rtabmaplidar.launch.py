import os
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

schedule = 1

def generate_launch_description():
    ld = LaunchDescription()

    tf = Node(package="tf2_ros", executable="static_transform_publisher",arguments=['-0.4','0','0','0','0','0','spot/base_link','os_sensor'],parameters=[{'use_sim_time': True}])
    ld.add_action(tf) 
    
    tf2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['0.3', '0', '0',  '1.5708', '0', '3.1416', 'spot/base_link', 'base_link'],
        parameters=[{'use_sim_time': True}]
    )
    
    imu = Node(
    package='imu_filter_madgwick', 
    executable='imu_filter_madgwick_node',
    parameters=[{
    'use_mag': False,
    "world_frame": "imu_link",
    "publish_tf": False
    }],
    remappings=[('imu/data_raw', 'imu/data_raw'),('imu/data', 'imu/filtered')]
    )
    ld.add_action(imu)
    
    slam_launch = os.path.join(
        get_package_share_directory("rtabmap_examples"), 'launch', "lidar3d.launch.py")
   
   
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch),launch_arguments={"lidar_topic": "/ouster/points", "frame_id": "os_sensor",}.items())) 
    
    ld.add_action(actions.ExecuteProcess( cmd=['ros2', 'bag', 'record', "--all"],output='screen' ))
    
    octomap = os.path.join(
        get_package_share_directory("octomap_server2"), 'launch', "octomap_server_launch.py")
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(octomap),launch_arguments={"input_cloud_topic": "/genz/local_map","frame_id": "odom", "base_frame_id": "os_lidar"}.items()))
    

    
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

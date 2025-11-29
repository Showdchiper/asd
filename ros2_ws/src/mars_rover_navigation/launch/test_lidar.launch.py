import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Object detector node
    object_detector = Node(
        package='mars_rover_navigation',
        executable='object_detector',
        name='object_detector',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_cluster_size': 3,
            'max_cluster_distance': 0.5,
            'min_object_distance': 0.2
        }],
        remappings=[
            ('/scan', '/lidar')  # Remap Gazebo lidar topic to ROS scan topic
        ],
        output='screen'
    )
    
    # Static transform: odom to base_link
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    # Static transform: base_link to lidar_link
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0', '0', '0.6', '0', '0', '0', 'base_link', 'lidar_link']
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time'),
        object_detector,
        static_tf_odom_to_base,
        static_tf_base_to_lidar
    ])

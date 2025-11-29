#!/usr/bin/env python3
"""
Mars Rover Exploration System Launch File
Launches object detection, path recording, habitat finding, and navigation
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Package directory
    pkg_mars_rover = FindPackageShare('mars_rover_navigation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_exploration = LaunchConfiguration('enable_exploration', default='true')
    
    # Data save directories
    data_base_path = '/home/sakshi/ignitio/ros2_ws/data'
    
    # Gazebo-ROS bridge for topics
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        output='screen'
    )
    
    # Python Object Detector Node
    object_detector_py = Node(
        package='mars_rover_navigation',
        executable='object_detector.py',
        name='object_detector_python',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_cluster_size': 3,
            'max_cluster_distance': 0.5,
            'min_object_distance': 0.2,
            'max_object_distance': 25.0,
            'rock_size_threshold': 0.8,
            'crater_size_threshold': 2.0,
            'wall_size_threshold': 3.0,
        }],
        output='screen'
    )
    
    # Path Recorder Node
    path_recorder = Node(
        package='mars_rover_navigation',
        executable='path_recorder.py',
        name='path_recorder',
        parameters=[{
            'use_sim_time': use_sim_time,
            'save_directory': f'{data_base_path}/paths',
            'save_interval': 30.0,
            'min_distance_threshold': 0.1,
            'max_path_points': 10000,
            'auto_save': True,
        }],
        output='screen'
    )
    
    # Habitat Finder Node
    habitat_finder = Node(
        package='mars_rover_navigation',
        executable='habitat_finder.py',
        name='habitat_finder',
        parameters=[{
            'use_sim_time': use_sim_time,
            'analysis_radius': 15.0,
            'min_flat_area': 5.0,
            'flatness_threshold': 0.3,
            'min_open_space': 0.4,
            'shelter_distance': 3.0,
            'save_directory': f'{data_base_path}/habitats',
            'update_interval': 2.0,
        }],
        output='screen'
    )
    
    # Navigation Controller Node
    navigation_controller = Node(
        package='mars_rover_navigation',
        executable='navigation_controller.py',
        name='navigation_controller',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_linear_speed': 0.5,
            'max_angular_speed': 1.0,
            'obstacle_distance_threshold': 1.5,
            'critical_distance': 0.5,
            'exploration_speed': 0.3,
            'goal_tolerance': 0.5,
            'scan_angle_front': 30.0,
            'scan_angle_side': 60.0,
        }],
        output='screen'
    )
    
    # Static transform: odom to base_link (simplified)
    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    # Static transform: base_link to lidar_link
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0', '0', '0.6', '0', '0', '0', 'base_link', 'lidar_link']
    )
    
    # Delayed start for exploration command (give time for nodes to initialize)
    start_exploration = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once', '/navigation_command', 
                     'std_msgs/msg/String', '{data: "explore"}'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time'),
        DeclareLaunchArgument('enable_exploration', default_value='true',
                            description='Start exploration automatically'),
        
        # Launch nodes
        gz_bridge,
        static_tf_odom_base,
        static_tf_base_to_lidar,
        object_detector_py,
        path_recorder,
        habitat_finder,
        navigation_controller,
        
        # Start exploration after delay (if enabled)
        start_exploration,
    ])

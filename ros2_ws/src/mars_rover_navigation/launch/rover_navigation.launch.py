import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Package directories
    pkg_mars_rover = FindPackageShare('mars_rover_navigation')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    
    # Paths to config files
    nav2_params_file = PathJoinSubstitution([pkg_mars_rover, 'config', 'nav2_params.yaml'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo-ROS bridge for topics
    # Bridge core topics (use gz.msgs.* types for Fortress)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # TF bridge optional; uncomment if needed
            # '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        output='screen'
    )
    
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
        output='screen'
    )
    
    # Robot state publisher for TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': get_robot_description()
        }],
        output='screen'
    )
    
    # Static transform: base_link to lidar_link
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0', '0', '0.6', '0', '0', '0', 'base_link', 'lidar_link']
    )
    
    # Nav2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time'),
        gz_bridge,
        object_detector,
        robot_state_publisher,
        static_tf_base_to_lidar,
        nav2_bringup_launch
    ])

def get_robot_description():
    return '''<?xml version="1.0"?>
<robot name="mars_rover">
  <link name="base_link"/>
  <link name="lidar_link"/>
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
  </joint>
</robot>'''

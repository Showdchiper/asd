#!/usr/bin/env python3
"""
Navigation Controller Node for Mars Rover
Controls rover navigation with obstacle avoidance and goal-seeking behavior
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool
import math
import random
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

class NavigationState(Enum):
    """Navigation state machine states"""
    IDLE = 0
    EXPLORING = 1
    NAVIGATING_TO_GOAL = 2
    AVOIDING_OBSTACLE = 3
    STOPPED = 4


class ObstacleLocation(Enum):
    """Location of detected obstacles"""
    NONE = 0
    LEFT = 1
    RIGHT = 2
    FRONT = 3
    FRONT_LEFT = 4
    FRONT_RIGHT = 5


@dataclass
class NavigationGoal:
    """Navigation goal point"""
    x: float
    y: float
    tolerance: float = 0.5


class NavigationControllerNode(Node):
    """
    ROS2 Node for autonomous rover navigation
    
    Features:
    - Obstacle avoidance using LiDAR
    - Goal-seeking navigation
    - Exploration mode
    - Integration with habitat finder
    
    Subscribes to:
        /lidar (sensor_msgs/LaserScan) - LiDAR data for obstacle detection
        /odom (nav_msgs/Odometry) - Rover position
        /best_habitat (geometry_msgs/PoseStamped) - Target habitat location
        /navigation_command (std_msgs/String) - Navigation commands
        
    Publishes:
        /cmd_vel (geometry_msgs/Twist) - Velocity commands
        /navigation_status (std_msgs/String) - Navigation status
        /navigation_markers (visualization_msgs/MarkerArray) - Visualization
    """
    
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Declare parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('obstacle_distance_threshold', 0.8)  # Reduced from 1.5
        self.declare_parameter('critical_distance', 0.3)  # Reduced from 0.5
        self.declare_parameter('exploration_speed', 0.3)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('scan_angle_front', 20.0)  # Narrower front angle (was 30)
        self.declare_parameter('scan_angle_side', 45.0)  # Reduced from 60
        self.declare_parameter('auto_start', True)  # Start exploring automatically
        
        # Get parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.exploration_speed = self.get_parameter('exploration_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.scan_angle_front = math.radians(self.get_parameter('scan_angle_front').value)
        self.scan_angle_side = math.radians(self.get_parameter('scan_angle_side').value)
        auto_start = self.get_parameter('auto_start').value
        
        # State - Start in EXPLORING mode if auto_start is True
        self.state = NavigationState.EXPLORING if auto_start else NavigationState.IDLE
        self.current_position: Optional[Tuple[float, float, float]] = None
        self.current_yaw: float = 0.0
        self.current_goal: Optional[NavigationGoal] = None
        self.latest_scan: Optional[LaserScan] = None
        self.obstacle_location = ObstacleLocation.NONE
        self.scan_received = False  # Track if we have LiDAR data
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/lidar', self.scan_callback, 10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        self.habitat_sub = self.create_subscription(
            PoseStamped, '/best_habitat', self.habitat_callback, 10
        )
        
        self.command_sub = self.create_subscription(
            String, '/navigation_command', self.command_callback, 10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/navigation_markers', 10)
        
        # Control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Status timer (2 Hz for more frequent status)
        self.status_timer = self.create_timer(0.5, self.publish_status)
        
        # Movement log counter
        self.move_count = 0
        self.explore_count = 0
        
        self.get_logger().info('🧭 Navigation Controller Node Started')
        self.get_logger().info(f'   Max linear speed: {self.max_linear_speed} m/s')
        self.get_logger().info(f'   Exploration speed: {self.exploration_speed} m/s')
        self.get_logger().info(f'   Auto-start exploration: {auto_start}')
        self.get_logger().info(f'   Initial state: {self.state.name}')
        
    def scan_callback(self, msg: LaserScan):
        """Process LiDAR scan for obstacle detection"""
        self.latest_scan = msg
        self.scan_received = True
        self.obstacle_location = self.detect_obstacles(msg)
        
    def odom_callback(self, msg: Odometry):
        """Update current position and orientation"""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def habitat_callback(self, msg: PoseStamped):
        """Receive best habitat location as potential goal"""
        if self.state == NavigationState.EXPLORING:
            # Auto-navigate to good habitats
            self.current_goal = NavigationGoal(
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                tolerance=1.0
            )
            self.state = NavigationState.NAVIGATING_TO_GOAL
            self.get_logger().info(
                f'🎯 New habitat goal: ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f})'
            )
            
    def command_callback(self, msg: String):
        """Process navigation commands"""
        command = msg.data.lower().strip()
        
        if command == 'start' or command == 'explore':
            self.state = NavigationState.EXPLORING
            self.get_logger().info('🚀 Starting exploration')
        elif command == 'stop':
            self.state = NavigationState.STOPPED
            self.stop_rover()
            self.get_logger().info('🛑 Navigation stopped')
        elif command == 'idle':
            self.state = NavigationState.IDLE
            self.stop_rover()
        elif command.startswith('goto'):
            # Parse goto command: "goto x y"
            try:
                parts = command.split()
                x, y = float(parts[1]), float(parts[2])
                self.current_goal = NavigationGoal(x=x, y=y)
                self.state = NavigationState.NAVIGATING_TO_GOAL
                self.get_logger().info(f'🎯 Navigating to ({x:.1f}, {y:.1f})')
            except (IndexError, ValueError):
                self.get_logger().error('Invalid goto command. Use: goto x y')
                
    def detect_obstacles(self, scan: LaserScan) -> ObstacleLocation:
        """Detect obstacles in different directions"""
        if scan is None:
            return ObstacleLocation.NONE
            
        num_readings = len(scan.ranges)
        center_idx = num_readings // 2
        
        # Calculate indices for different regions
        front_range = int(self.scan_angle_front / scan.angle_increment)
        side_range = int(self.scan_angle_side / scan.angle_increment)
        
        # Get ranges for each region
        def get_min_range(start_idx, end_idx):
            ranges = []
            for i in range(max(0, start_idx), min(num_readings, end_idx)):
                r = scan.ranges[i]
                if not (math.isnan(r) or math.isinf(r)):
                    if scan.range_min < r < scan.range_max:
                        ranges.append(r)
            return min(ranges) if ranges else float('inf')
        
        # Front region
        front_min = get_min_range(center_idx - front_range, center_idx + front_range)
        
        # Left region
        left_min = get_min_range(center_idx + front_range, center_idx + side_range)
        
        # Right region
        right_min = get_min_range(center_idx - side_range, center_idx - front_range)
        
        # Front-left
        front_left_min = get_min_range(
            center_idx + front_range // 2,
            center_idx + front_range + side_range // 2
        )
        
        # Front-right
        front_right_min = get_min_range(
            center_idx - front_range - side_range // 2,
            center_idx - front_range // 2
        )
        
        # Determine obstacle location
        if front_min < self.critical_distance:
            if front_left_min < front_right_min:
                return ObstacleLocation.FRONT_LEFT
            else:
                return ObstacleLocation.FRONT_RIGHT
        elif front_min < self.obstacle_threshold:
            return ObstacleLocation.FRONT
        elif left_min < self.obstacle_threshold:
            return ObstacleLocation.LEFT
        elif right_min < self.obstacle_threshold:
            return ObstacleLocation.RIGHT
        elif front_left_min < self.obstacle_threshold:
            return ObstacleLocation.FRONT_LEFT
        elif front_right_min < self.obstacle_threshold:
            return ObstacleLocation.FRONT_RIGHT
            
        return ObstacleLocation.NONE
        
    def control_loop(self):
        """Main control loop - runs at 10Hz"""
        # Log first 10 calls to verify timer is working
        if not hasattr(self, 'loop_count'):
            self.loop_count = 0
        self.loop_count += 1
        if self.loop_count <= 10:
            self.get_logger().info(f'⏱️ Control loop #{self.loop_count}, state={self.state.name}')
        
        if self.state == NavigationState.IDLE:
            self.stop_rover()
            return
            
        if self.state == NavigationState.STOPPED:
            self.stop_rover()
            return
        
        # Always try to move in exploring mode
        if self.state == NavigationState.EXPLORING:
            self.explore()
            return
            
        # Check for obstacles first (safety) when navigating to goal
        if self.obstacle_location != ObstacleLocation.NONE:
            self.avoid_obstacle()
            return
            
        if self.state == NavigationState.NAVIGATING_TO_GOAL:
            self.navigate_to_goal()
            
    def explore(self):
        """Exploration behavior - move forward while avoiding obstacles"""
        cmd = Twist()
        
        # Always move forward by default
        cmd.linear.x = self.exploration_speed
        cmd.angular.z = random.uniform(-0.1, 0.1)
        
        # Check for obstacles and avoid them
        if self.obstacle_location in [ObstacleLocation.FRONT, 
                                       ObstacleLocation.FRONT_LEFT,
                                       ObstacleLocation.FRONT_RIGHT]:
            # Obstacle in front - turn away
            if self.obstacle_location in [ObstacleLocation.FRONT_LEFT, ObstacleLocation.LEFT]:
                cmd.linear.x = 0.0
                cmd.angular.z = -self.max_angular_speed * 0.7  # Turn right
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.max_angular_speed * 0.7  # Turn left
            self.get_logger().warn(f'🚧 Obstacle {self.obstacle_location.name} - Turning!')
        elif self.obstacle_location == ObstacleLocation.LEFT:
            # Obstacle on left - slight turn right while moving
            cmd.linear.x = self.exploration_speed * 0.7
            cmd.angular.z = -self.max_angular_speed * 0.3
        elif self.obstacle_location == ObstacleLocation.RIGHT:
            # Obstacle on right - slight turn left while moving
            cmd.linear.x = self.exploration_speed * 0.7
            cmd.angular.z = self.max_angular_speed * 0.3
            
        # ALWAYS publish velocity command
        self.cmd_vel_pub.publish(cmd)
        
        # Log every 50th call (5 seconds at 10Hz)
        self.explore_count += 1
        if self.explore_count % 50 == 0:
            self.get_logger().info(f'🚗 Moving: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}')
        
    def navigate_to_goal(self):
        """Navigate to current goal"""
        if self.current_goal is None or self.current_position is None:
            self.state = NavigationState.EXPLORING
            return
            
        # Calculate distance and angle to goal
        dx = self.current_goal.x - self.current_position[0]
        dy = self.current_goal.y - self.current_position[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if we reached the goal
        if distance < self.current_goal.tolerance:
            self.get_logger().info('✅ Reached goal!')
            self.current_goal = None
            self.state = NavigationState.EXPLORING
            self.stop_rover()
            return
            
        # Calculate desired heading
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - self.current_yaw)
        
        cmd = Twist()
        
        # Angular control - turn towards goal
        if abs(yaw_error) > 0.1:
            cmd.angular.z = max(-self.max_angular_speed,
                               min(self.max_angular_speed, yaw_error * 2.0))
            # Slow down linear speed when turning
            cmd.linear.x = self.max_linear_speed * max(0.1, 1.0 - abs(yaw_error))
        else:
            # Facing goal - move forward
            cmd.linear.x = min(self.max_linear_speed, distance * 0.5)
            cmd.angular.z = yaw_error * 1.0
            
        self.cmd_vel_pub.publish(cmd)
        
    def avoid_obstacle(self):
        """Obstacle avoidance behavior"""
        cmd = Twist()
        
        if self.obstacle_location in [ObstacleLocation.FRONT, 
                                       ObstacleLocation.FRONT_LEFT,
                                       ObstacleLocation.FRONT_RIGHT]:
            # Stop forward motion
            cmd.linear.x = -0.1  # Slight reverse
            
            # Turn away from obstacle
            if self.obstacle_location in [ObstacleLocation.FRONT_LEFT, ObstacleLocation.LEFT]:
                cmd.angular.z = -self.max_angular_speed  # Turn right
            else:
                cmd.angular.z = self.max_angular_speed  # Turn left
                
        elif self.obstacle_location == ObstacleLocation.LEFT:
            cmd.linear.x = self.exploration_speed * 0.5
            cmd.angular.z = -self.max_angular_speed * 0.5
            
        elif self.obstacle_location == ObstacleLocation.RIGHT:
            cmd.linear.x = self.exploration_speed * 0.5
            cmd.angular.z = self.max_angular_speed * 0.5
            
        self.cmd_vel_pub.publish(cmd)
        
    def stop_rover(self):
        """Stop the rover"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def publish_status(self):
        """Publish navigation status with logging"""
        status = {
            'state': self.state.name,
            'obstacle': self.obstacle_location.name,
            'position': self.current_position,
            'yaw': self.current_yaw,
            'goal': {
                'x': self.current_goal.x,
                'y': self.current_goal.y
            } if self.current_goal else None
        }
        
        msg = String()
        msg.data = str(status)
        self.status_pub.publish(msg)
        
        # Log status periodically
        self.move_count += 1
        if self.move_count % 4 == 0:  # Log every 2 seconds (0.5s * 4)
            pos_str = f"({self.current_position[0]:.2f}, {self.current_position[1]:.2f})" if self.current_position else "Unknown"
            self.get_logger().info(f'📍 State: {self.state.name} | Pos: {pos_str} | Obstacle: {self.obstacle_location.name}')
        
        # Publish visualization
        self.publish_visualization()
        
    def publish_visualization(self):
        """Publish navigation visualization markers"""
        marker_array = MarkerArray()
        
        # Current position marker
        if self.current_position:
            pos_marker = Marker()
            pos_marker.header.stamp = self.get_clock().now().to_msg()
            pos_marker.header.frame_id = 'odom'
            pos_marker.ns = "navigation"
            pos_marker.id = 0
            pos_marker.type = Marker.ARROW
            pos_marker.action = Marker.ADD
            pos_marker.pose.position.x = self.current_position[0]
            pos_marker.pose.position.y = self.current_position[1]
            pos_marker.pose.position.z = 0.5
            
            # Set orientation from yaw
            pos_marker.pose.orientation.z = math.sin(self.current_yaw / 2)
            pos_marker.pose.orientation.w = math.cos(self.current_yaw / 2)
            
            pos_marker.scale.x = 1.0
            pos_marker.scale.y = 0.2
            pos_marker.scale.z = 0.2
            pos_marker.color.r = 0.0
            pos_marker.color.g = 0.5
            pos_marker.color.b = 1.0
            pos_marker.color.a = 1.0
            
            marker_array.markers.append(pos_marker)
            
        # Goal marker
        if self.current_goal:
            goal_marker = Marker()
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.header.frame_id = 'odom'
            goal_marker.ns = "navigation"
            goal_marker.id = 1
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = self.current_goal.x
            goal_marker.pose.position.y = self.current_goal.y
            goal_marker.pose.position.z = 0.5
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 1.0
            goal_marker.scale.y = 1.0
            goal_marker.scale.z = 1.0
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.8
            goal_marker.color.b = 0.0
            goal_marker.color.a = 0.8
            
            marker_array.markers.append(goal_marker)
            
            # Line to goal
            if self.current_position:
                line_marker = Marker()
                line_marker.header = goal_marker.header
                line_marker.ns = "navigation"
                line_marker.id = 2
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                line_marker.scale.x = 0.1
                line_marker.color.r = 1.0
                line_marker.color.g = 1.0
                line_marker.color.b = 0.0
                line_marker.color.a = 0.5
                
                p1 = Point()
                p1.x = self.current_position[0]
                p1.y = self.current_position[1]
                p1.z = 0.3
                
                p2 = Point()
                p2.x = self.current_goal.x
                p2.y = self.current_goal.y
                p2.z = 0.3
                
                line_marker.points = [p1, p2]
                marker_array.markers.append(line_marker)
                
        # State text
        state_marker = Marker()
        state_marker.header.stamp = self.get_clock().now().to_msg()
        state_marker.header.frame_id = 'odom'
        state_marker.ns = "navigation"
        state_marker.id = 10
        state_marker.type = Marker.TEXT_VIEW_FACING
        state_marker.action = Marker.ADD
        
        if self.current_position:
            state_marker.pose.position.x = self.current_position[0]
            state_marker.pose.position.y = self.current_position[1]
            state_marker.pose.position.z = 2.0
        
        state_marker.scale.z = 0.5
        state_marker.color.r = 1.0
        state_marker.color.g = 1.0
        state_marker.color.b = 1.0
        state_marker.color.a = 1.0
        state_marker.text = f"State: {self.state.name}\nObstacle: {self.obstacle_location.name}"
        
        marker_array.markers.append(state_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_rover()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

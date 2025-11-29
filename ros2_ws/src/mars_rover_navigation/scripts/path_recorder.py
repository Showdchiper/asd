#!/usr/bin/env python3
"""
Path Recorder Node for Mars Rover
Records the rover's travel path and saves it to files
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import json
import os
import math
from datetime import datetime
from typing import List, Tuple, Dict
from dataclasses import dataclass, asdict
import csv


@dataclass
class PathPoint:
    """Single point in the recorded path"""
    x: float
    y: float
    z: float
    timestamp: float
    linear_velocity: float
    angular_velocity: float


class PathRecorderNode(Node):
    """
    ROS2 Node for recording the rover's travel path
    
    Subscribes to:
        /odom (nav_msgs/Odometry) - Rover odometry data
        
    Publishes:
        /recorded_path (nav_msgs/Path) - Visualization of recorded path
        /path_markers (visualization_msgs/MarkerArray) - Path markers
    """
    
    def __init__(self):
        super().__init__('path_recorder')
        
        # Declare parameters
        self.declare_parameter('save_directory', '/home/sakshi/ignitio/ros2_ws/data/paths')
        self.declare_parameter('save_interval', 10.0)  # Save every 10 seconds
        self.declare_parameter('min_distance_threshold', 0.1)  # Minimum distance to record new point
        self.declare_parameter('max_path_points', 10000)
        self.declare_parameter('auto_save', True)
        
        # Get parameters
        self.save_directory = self.get_parameter('save_directory').value
        self.save_interval = self.get_parameter('save_interval').value
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').value
        self.max_path_points = self.get_parameter('max_path_points').value
        self.auto_save = self.get_parameter('auto_save').value
        
        # Create save directory if it doesn't exist
        os.makedirs(self.save_directory, exist_ok=True)
        
        # Path storage
        self.path_points: List[PathPoint] = []
        self.last_position: Tuple[float, float, float] = None
        self.total_distance: float = 0.0
        self.start_time: float = None
        
        # Session info
        self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(
            Path,
            '/recorded_path',
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/path_markers',
            10
        )
        
        # Timer for auto-save
        if self.auto_save:
            self.save_timer = self.create_timer(
                self.save_interval,
                self.auto_save_callback
            )
            
        # Timer for publishing path visualization
        self.viz_timer = self.create_timer(1.0, self.publish_path_visualization)
        
        # Timer to log path stats
        self.stats_timer = self.create_timer(5.0, self.log_path_stats)
        
        self.get_logger().info('📍 Path Recorder Node Started')
        self.get_logger().info(f'   Save directory: {self.save_directory}')
        self.get_logger().info(f'   Session ID: {self.session_id}')
        
    def log_path_stats(self):
        """Log path statistics periodically"""
        if self.path_points:
            self.get_logger().info(f'📍 Path: {len(self.path_points)} points, {self.total_distance:.2f}m traveled')
        
    def odom_callback(self, msg: Odometry):
        """Process incoming odometry data"""
        current_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
        # Initialize start time
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f'📍 Started recording path at ({current_pos[0]:.2f}, {current_pos[1]:.2f})')
            
        # Check if we should record this point
        if self.should_record_point(current_pos):
            # Calculate velocities
            linear_vel = math.sqrt(
                msg.twist.twist.linear.x**2 +
                msg.twist.twist.linear.y**2 +
                msg.twist.twist.linear.z**2
            )
            angular_vel = msg.twist.twist.angular.z
            
            # Create path point
            timestamp = self.get_clock().now().nanoseconds / 1e9
            point = PathPoint(
                x=current_pos[0],
                y=current_pos[1],
                z=current_pos[2],
                timestamp=timestamp,
                linear_velocity=linear_vel,
                angular_velocity=angular_vel
            )
            
            # Add to path
            self.path_points.append(point)
            
            # Update total distance
            if self.last_position:
                self.total_distance += self.calculate_distance(
                    self.last_position, current_pos
                )
                
            self.last_position = current_pos
            
            # Limit path size
            if len(self.path_points) > self.max_path_points:
                self.path_points = self.path_points[-self.max_path_points:]
                
    def should_record_point(self, current_pos: Tuple[float, float, float]) -> bool:
        """Check if we should record this position"""
        if self.last_position is None:
            return True
            
        distance = self.calculate_distance(self.last_position, current_pos)
        return distance >= self.min_distance_threshold
        
    def calculate_distance(self, pos1: Tuple[float, float, float], 
                          pos2: Tuple[float, float, float]) -> float:
        """Calculate 3D distance between two positions"""
        return math.sqrt(
            (pos2[0] - pos1[0])**2 +
            (pos2[1] - pos1[1])**2 +
            (pos2[2] - pos1[2])**2
        )
        
    def publish_path_visualization(self):
        """Publish path visualization"""
        if not self.path_points:
            return
            
        # Publish Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        for point in self.path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        
        # Publish markers
        marker_array = MarkerArray()
        
        # Line strip for path
        line_marker = Marker()
        line_marker.header = path_msg.header
        line_marker.ns = "path_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1
        line_marker.color.r = 0.0
        line_marker.color.g = 0.7
        line_marker.color.b = 1.0
        line_marker.color.a = 0.8
        
        for point in self.path_points[-500:]:  # Last 500 points
            p = Point()
            p.x = point.x
            p.y = point.y
            p.z = point.z + 0.1
            line_marker.points.append(p)
            
        marker_array.markers.append(line_marker)
        
        # Start point marker
        if self.path_points:
            start_marker = Marker()
            start_marker.header = path_msg.header
            start_marker.ns = "path_endpoints"
            start_marker.id = 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose.position.x = self.path_points[0].x
            start_marker.pose.position.y = self.path_points[0].y
            start_marker.pose.position.z = self.path_points[0].z + 0.5
            start_marker.scale.x = 0.5
            start_marker.scale.y = 0.5
            start_marker.scale.z = 0.5
            start_marker.color.r = 0.0
            start_marker.color.g = 1.0
            start_marker.color.b = 0.0
            start_marker.color.a = 1.0
            marker_array.markers.append(start_marker)
            
            # Current position marker
            end_marker = Marker()
            end_marker.header = path_msg.header
            end_marker.ns = "path_endpoints"
            end_marker.id = 2
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            end_marker.pose.position.x = self.path_points[-1].x
            end_marker.pose.position.y = self.path_points[-1].y
            end_marker.pose.position.z = self.path_points[-1].z + 0.5
            end_marker.scale.x = 0.5
            end_marker.scale.y = 0.5
            end_marker.scale.z = 0.5
            end_marker.color.r = 1.0
            end_marker.color.g = 0.0
            end_marker.color.b = 0.0
            end_marker.color.a = 1.0
            marker_array.markers.append(end_marker)
            
        self.marker_pub.publish(marker_array)
        
    def auto_save_callback(self):
        """Automatically save path periodically"""
        if self.path_points:
            self.save_path()
            
    def save_path(self, filename: str = None):
        """Save the recorded path to files"""
        if not self.path_points:
            self.get_logger().warn('No path points to save')
            return
            
        if filename is None:
            filename = f"path_{self.session_id}"
            
        # Save as JSON
        json_path = os.path.join(self.save_directory, f"{filename}.json")
        path_data = {
            'session_id': self.session_id,
            'total_distance': self.total_distance,
            'num_points': len(self.path_points),
            'start_time': self.start_time,
            'duration': (self.get_clock().now().nanoseconds / 1e9) - self.start_time if self.start_time else 0,
            'points': [asdict(p) for p in self.path_points]
        }
        
        with open(json_path, 'w') as f:
            json.dump(path_data, f, indent=2)
            
        # Save as CSV
        csv_path = os.path.join(self.save_directory, f"{filename}.csv")
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'linear_velocity', 'angular_velocity'])
            for point in self.path_points:
                writer.writerow([
                    point.timestamp, point.x, point.y, point.z,
                    point.linear_velocity, point.angular_velocity
                ])
                
        self.get_logger().info(f'💾 Path saved: {len(self.path_points)} points, {self.total_distance:.2f}m total')
        
    def get_path_statistics(self) -> Dict:
        """Get statistics about the recorded path"""
        if not self.path_points:
            return {}
            
        velocities = [p.linear_velocity for p in self.path_points]
        
        return {
            'total_distance': self.total_distance,
            'num_points': len(self.path_points),
            'avg_velocity': sum(velocities) / len(velocities) if velocities else 0,
            'max_velocity': max(velocities) if velocities else 0,
            'duration': (self.get_clock().now().nanoseconds / 1e9) - self.start_time if self.start_time else 0
        }


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save path before shutdown
        node.save_path()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

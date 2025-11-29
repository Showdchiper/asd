#!/usr/bin/env python3
"""
Object Detector Node for Mars Rover
Detects objects using LiDAR sensor data and publishes detected objects
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
import numpy as np
import math
import json
from typing import List, Tuple
from dataclasses import dataclass, asdict
from enum import Enum


class ObjectType(Enum):
    """Classification of detected objects"""
    UNKNOWN = 0
    ROCK = 1
    CRATER = 2
    WALL = 3
    POTENTIAL_HABITAT = 4


@dataclass
class ClusterInfo:
    """Information about a detected cluster/object"""
    points: List[Tuple[float, float]]
    centroid: Tuple[float, float]
    size: float
    object_type: ObjectType
    distance: float
    angle: float


class ObjectDetectorNode(Node):
    """
    ROS2 Node for detecting objects using LiDAR data
    
    Subscribes to:
        /lidar (sensor_msgs/LaserScan) - LiDAR scan data
        
    Publishes:
        /detected_objects (visualization_msgs/MarkerArray) - Visualization markers
        /detected_objects_data (std_msgs/String) - JSON object data for navigation
    """
    
    def __init__(self):
        super().__init__('object_detector_python')
        
        # Declare parameters
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('max_cluster_distance', 0.5)
        self.declare_parameter('min_object_distance', 0.2)
        self.declare_parameter('max_object_distance', 25.0)
        self.declare_parameter('rock_size_threshold', 0.8)
        self.declare_parameter('crater_size_threshold', 2.0)
        self.declare_parameter('wall_size_threshold', 3.0)
        self.declare_parameter('habitat_min_distance', 5.0)
        
        # Get parameters
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_distance = self.get_parameter('max_cluster_distance').value
        self.min_object_distance = self.get_parameter('min_object_distance').value
        self.max_object_distance = self.get_parameter('max_object_distance').value
        self.rock_size_threshold = self.get_parameter('rock_size_threshold').value
        self.crater_size_threshold = self.get_parameter('crater_size_threshold').value
        self.wall_size_threshold = self.get_parameter('wall_size_threshold').value
        self.habitat_min_distance = self.get_parameter('habitat_min_distance').value
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar',
            self.scan_callback,
            10
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/detected_objects',
            10
        )
        
        self.objects_pub = self.create_publisher(
            MarkerArray,
            '/detected_objects_markers',
            10
        )
        
        # Publisher for object data as JSON
        self.objects_data_pub = self.create_publisher(
            String,
            '/detected_objects_data',
            10
        )
        
        # Store detected objects
        self.detected_objects: List[ClusterInfo] = []
        self.object_history: List[ClusterInfo] = []
        
        self.get_logger().info('🔍 Object Detector Node (Python) Started')
        self.get_logger().info(f'   Min cluster size: {self.min_cluster_size}')
        self.get_logger().info(f'   Max cluster distance: {self.max_cluster_distance}m')
        self.scan_count = 0
        
    def scan_callback(self, msg: LaserScan):
        """Process incoming LiDAR scan data"""
        self.scan_count += 1
        
        # Convert scan to Cartesian points
        points = self.scan_to_points(msg)
        
        if len(points) < self.min_cluster_size:
            return
            
        # Cluster the points
        clusters = self.cluster_points(points, msg)
        
        # Classify objects
        self.detected_objects = []
        for cluster in clusters:
            obj_info = self.classify_object(cluster)
            if obj_info:
                self.detected_objects.append(obj_info)
                
        # Publish visualization markers
        self.publish_markers(msg.header)
        
        # Publish object data as JSON
        self.publish_objects_data()
        
        # Log detected objects periodically (every 40 scans = ~5 seconds at 8Hz)
        if self.scan_count % 40 == 0:
            obj_counts = {}
            for obj in self.detected_objects:
                obj_counts[obj.object_type.name] = obj_counts.get(obj.object_type.name, 0) + 1
            if obj_counts:
                self.get_logger().info(f'🔍 Detected: {obj_counts}')
            
    def publish_objects_data(self):
        """Publish detected objects as JSON data"""
        objects_data = [{
            'type': obj.object_type.name,
            'centroid': {'x': obj.centroid[0], 'y': obj.centroid[1]},
            'size': obj.size,
            'distance': obj.distance,
            'angle': obj.angle
        } for obj in self.detected_objects]
        
        msg = String()
        msg.data = json.dumps(objects_data)
        self.objects_data_pub.publish(msg)
            
    def scan_to_points(self, msg: LaserScan) -> List[Tuple[float, float, int]]:
        """Convert LaserScan to list of (x, y, index) points"""
        points = []
        
        for i, range_val in enumerate(msg.ranges):
            # Skip invalid readings
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
            if range_val < self.min_object_distance or range_val > self.max_object_distance:
                continue
                
            # Convert polar to Cartesian
            angle = msg.angle_min + i * msg.angle_increment
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            
            points.append((x, y, i))
            
        return points
        
    def cluster_points(self, points: List[Tuple[float, float, int]], msg: LaserScan) -> List[List[Tuple[float, float]]]:
        """Cluster nearby points into objects"""
        if not points:
            return []
            
        clusters = []
        current_cluster = [points[0][:2]]  # Take only x, y
        
        for i in range(1, len(points)):
            prev_point = points[i-1]
            curr_point = points[i]
            
            # Calculate distance between consecutive points
            distance = math.sqrt(
                (curr_point[0] - prev_point[0])**2 + 
                (curr_point[1] - prev_point[1])**2
            )
            
            if distance < self.max_cluster_distance:
                # Add to current cluster
                current_cluster.append(curr_point[:2])
            else:
                # Save cluster if large enough
                if len(current_cluster) >= self.min_cluster_size:
                    clusters.append(current_cluster)
                # Start new cluster
                current_cluster = [curr_point[:2]]
                
        # Don't forget the last cluster
        if len(current_cluster) >= self.min_cluster_size:
            clusters.append(current_cluster)
            
        return clusters
        
    def classify_object(self, cluster: List[Tuple[float, float]]) -> ClusterInfo:
        """Classify a cluster as a specific object type"""
        if not cluster:
            return None
            
        # Calculate centroid
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        centroid = (sum(xs) / len(xs), sum(ys) / len(ys))
        
        # Calculate size (bounding box diagonal)
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        width = max_x - min_x
        height = max_y - min_y
        size = math.sqrt(width**2 + height**2)
        
        # Calculate distance and angle from rover
        distance = math.sqrt(centroid[0]**2 + centroid[1]**2)
        angle = math.atan2(centroid[1], centroid[0])
        
        # Classify based on size and shape
        aspect_ratio = max(width, height) / (min(width, height) + 0.001)
        
        if size < self.rock_size_threshold:
            obj_type = ObjectType.ROCK
        elif size < self.crater_size_threshold:
            if aspect_ratio < 1.5:
                obj_type = ObjectType.CRATER
            else:
                obj_type = ObjectType.ROCK
        elif size < self.wall_size_threshold:
            if aspect_ratio > 3.0:
                obj_type = ObjectType.WALL
            else:
                obj_type = ObjectType.POTENTIAL_HABITAT
        else:
            if aspect_ratio > 4.0:
                obj_type = ObjectType.WALL
            else:
                obj_type = ObjectType.POTENTIAL_HABITAT
                
        return ClusterInfo(
            points=cluster,
            centroid=centroid,
            size=size,
            object_type=obj_type,
            distance=distance,
            angle=angle
        )
        
    def publish_markers(self, header):
        """Publish visualization markers for detected objects"""
        marker_array = MarkerArray()
        
        # Delete old markers
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, obj in enumerate(self.detected_objects):
            # Create marker for each object
            marker = Marker()
            marker.header = header
            marker.ns = "detected_objects"
            marker.id = i + 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Position at centroid
            marker.pose.position.x = obj.centroid[0]
            marker.pose.position.y = obj.centroid[1]
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            
            # Size based on object size
            marker.scale.x = max(0.2, obj.size)
            marker.scale.y = max(0.2, obj.size)
            marker.scale.z = 0.5
            
            # Color based on object type
            marker.color = self.get_object_color(obj.object_type)
            marker.lifetime.sec = 1
            
            marker_array.markers.append(marker)
            
            # Add text label
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "object_labels"
            text_marker.id = i + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = obj.centroid[0]
            text_marker.pose.position.y = obj.centroid[1]
            text_marker.pose.position.z = 1.0
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{obj.object_type.name}\n{obj.distance:.1f}m"
            text_marker.lifetime.sec = 1
            
            marker_array.markers.append(text_marker)
            
        self.marker_pub.publish(marker_array)
        self.objects_pub.publish(marker_array)
        
    def get_object_color(self, obj_type: ObjectType) -> ColorRGBA:
        """Get color for object type visualization"""
        color = ColorRGBA()
        color.a = 0.8
        
        if obj_type == ObjectType.ROCK:
            color.r, color.g, color.b = 0.6, 0.4, 0.2  # Brown
        elif obj_type == ObjectType.CRATER:
            color.r, color.g, color.b = 0.3, 0.3, 0.3  # Gray
        elif obj_type == ObjectType.WALL:
            color.r, color.g, color.b = 0.8, 0.2, 0.2  # Red
        elif obj_type == ObjectType.POTENTIAL_HABITAT:
            color.r, color.g, color.b = 0.2, 0.8, 0.2  # Green
        else:
            color.r, color.g, color.b = 1.0, 1.0, 0.0  # Yellow
            
        return color
        
    def get_detected_objects(self) -> List[ClusterInfo]:
        """Return list of currently detected objects"""
        return self.detected_objects


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Habitat Finder Node for Mars Rover
Analyzes terrain and detected objects to find suitable habitat locations
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
import json
import os
import math
import numpy as np
from datetime import datetime
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass, asdict
from enum import Enum


class HabitatQuality(Enum):
    """Quality rating for potential habitats"""
    UNSUITABLE = 0
    POOR = 1
    FAIR = 2
    GOOD = 3
    EXCELLENT = 4


@dataclass
class HabitatCandidate:
    """Potential habitat location"""
    x: float
    y: float
    z: float
    quality: HabitatQuality
    score: float
    flatness_score: float
    shelter_score: float
    accessibility_score: float
    area_score: float
    timestamp: float
    visited: bool = False


@dataclass
class TerrainAnalysis:
    """Terrain analysis at a specific location"""
    position: Tuple[float, float]
    flatness: float  # 0-1, 1 is perfectly flat
    roughness: float  # 0-1, 0 is smooth
    obstacle_density: float  # obstacles per unit area
    open_space: float  # percentage of open space
    shelter_nearby: bool


class HabitatFinderNode(Node):
    """
    ROS2 Node for finding suitable habitat locations on Mars
    
    Criteria for good habitat:
    - Flat terrain (low variance in LiDAR readings)
    - Some shelter from nearby structures
    - Accessible area (not blocked by obstacles)
    - Sufficient open space
    
    Subscribes to:
        /lidar (sensor_msgs/LaserScan) - LiDAR scan data
        /odom (nav_msgs/Odometry) - Rover position
        
    Publishes:
        /habitat_markers (visualization_msgs/MarkerArray) - Habitat visualization
        /habitat_candidates (std_msgs/String) - JSON list of candidates
        /best_habitat (geometry_msgs/PoseStamped) - Best habitat location
    """
    
    def __init__(self):
        super().__init__('habitat_finder')
        
        # Declare parameters
        self.declare_parameter('analysis_radius', 15.0)  # meters
        self.declare_parameter('min_flat_area', 5.0)  # minimum flat area size
        self.declare_parameter('flatness_threshold', 0.3)  # max variance for flat
        self.declare_parameter('min_open_space', 0.4)  # minimum 40% open
        self.declare_parameter('shelter_distance', 3.0)  # ideal shelter distance
        self.declare_parameter('save_directory', '/home/sakshi/ignitio/ros2_ws/data/habitats')
        self.declare_parameter('update_interval', 2.0)  # analysis update interval
        
        # Get parameters
        self.analysis_radius = self.get_parameter('analysis_radius').value
        self.min_flat_area = self.get_parameter('min_flat_area').value
        self.flatness_threshold = self.get_parameter('flatness_threshold').value
        self.min_open_space = self.get_parameter('min_open_space').value
        self.shelter_distance = self.get_parameter('shelter_distance').value
        self.save_directory = self.get_parameter('save_directory').value
        self.update_interval = self.get_parameter('update_interval').value
        
        # Create save directory
        os.makedirs(self.save_directory, exist_ok=True)
        
        # Storage
        self.habitat_candidates: List[HabitatCandidate] = []
        self.current_position: Optional[Tuple[float, float, float]] = None
        self.latest_scan: Optional[LaserScan] = None
        self.terrain_map: Dict[Tuple[int, int], TerrainAnalysis] = {}
        self.grid_resolution = 1.0  # 1 meter grid cells
        
        # Session info
        self.session_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar',
            self.scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/habitat_markers',
            10
        )
        
        self.candidates_pub = self.create_publisher(
            String,
            '/habitat_candidates',
            10
        )
        
        self.best_habitat_pub = self.create_publisher(
            PoseStamped,
            '/best_habitat',
            10
        )
        
        # Analysis timer
        self.analysis_timer = self.create_timer(
            self.update_interval,
            self.analyze_terrain
        )
        
        # Visualization timer
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)
        
        self.get_logger().info('🏠 Habitat Finder Node Started')
        self.get_logger().info(f'   Analysis radius: {self.analysis_radius}m')
        self.get_logger().info(f'   Flatness threshold: {self.flatness_threshold}')
        
    def scan_callback(self, msg: LaserScan):
        """Store latest LiDAR scan"""
        self.latest_scan = msg
        
    def odom_callback(self, msg: Odometry):
        """Update current position"""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
    def analyze_terrain(self):
        """Analyze current terrain for habitat suitability"""
        if self.latest_scan is None or self.current_position is None:
            return
            
        scan = self.latest_scan
        
        # Convert scan to sectors for analysis
        sectors = self.analyze_scan_sectors(scan)
        
        # Find potential habitat areas
        for sector in sectors:
            if self.is_potential_habitat(sector):
                candidate = self.create_habitat_candidate(sector)
                self.add_habitat_candidate(candidate)
                
        # Update terrain map
        self.update_terrain_map(scan)
        
        # Publish best habitat
        self.publish_best_habitat()
        
    def analyze_scan_sectors(self, scan: LaserScan) -> List[Dict]:
        """Divide scan into sectors and analyze each"""
        num_sectors = 12  # Divide 360° into 12 sectors (30° each)
        sector_size = len(scan.ranges) // num_sectors
        sectors = []
        
        for i in range(num_sectors):
            start_idx = i * sector_size
            end_idx = start_idx + sector_size
            
            sector_ranges = scan.ranges[start_idx:end_idx]
            valid_ranges = [r for r in sector_ranges 
                          if not (math.isnan(r) or math.isinf(r))
                          and scan.range_min < r < scan.range_max]
            
            if not valid_ranges:
                continue
                
            # Calculate sector statistics
            avg_range = np.mean(valid_ranges)
            variance = np.var(valid_ranges)
            min_range = min(valid_ranges)
            max_range = max(valid_ranges)
            
            # Sector center angle
            center_angle = scan.angle_min + (start_idx + sector_size/2) * scan.angle_increment
            
            # Calculate centroid of sector
            centroid_x = self.current_position[0] + avg_range * math.cos(center_angle)
            centroid_y = self.current_position[1] + avg_range * math.sin(center_angle)
            
            sectors.append({
                'angle': center_angle,
                'avg_range': avg_range,
                'variance': variance,
                'min_range': min_range,
                'max_range': max_range,
                'centroid': (centroid_x, centroid_y),
                'num_points': len(valid_ranges),
                'open_ratio': len([r for r in valid_ranges if r > self.analysis_radius * 0.5]) / len(valid_ranges)
            })
            
        return sectors
        
    def is_potential_habitat(self, sector: Dict) -> bool:
        """Check if sector could be a habitat location"""
        # Check flatness (low variance)
        if sector['variance'] > self.flatness_threshold:
            return False
            
        # Check open space
        if sector['open_ratio'] < self.min_open_space:
            return False
            
        # Check minimum distance (not too close to rover)
        if sector['avg_range'] < 2.0:
            return False
            
        return True
        
    def create_habitat_candidate(self, sector: Dict) -> HabitatCandidate:
        """Create a habitat candidate from sector analysis"""
        # Calculate individual scores
        flatness_score = 1.0 - min(1.0, sector['variance'] / self.flatness_threshold)
        
        # Open space score
        accessibility_score = sector['open_ratio']
        
        # Shelter score - some obstacles nearby is good for shelter
        shelter_distance = sector['min_range']
        if shelter_distance < 1.0:
            shelter_score = 0.2  # Too close
        elif shelter_distance < self.shelter_distance:
            shelter_score = 0.8  # Good shelter nearby
        elif shelter_distance < self.shelter_distance * 2:
            shelter_score = 0.5  # Moderate shelter
        else:
            shelter_score = 0.3  # No shelter
            
        # Area score based on range variance (larger open areas = better)
        area_range = sector['max_range'] - sector['min_range']
        area_score = max(0, 1.0 - area_range / 10.0)
        
        # Calculate total score
        total_score = (
            flatness_score * 0.35 +
            shelter_score * 0.25 +
            accessibility_score * 0.25 +
            area_score * 0.15
        )
        
        # Determine quality
        if total_score >= 0.8:
            quality = HabitatQuality.EXCELLENT
        elif total_score >= 0.6:
            quality = HabitatQuality.GOOD
        elif total_score >= 0.4:
            quality = HabitatQuality.FAIR
        elif total_score >= 0.2:
            quality = HabitatQuality.POOR
        else:
            quality = HabitatQuality.UNSUITABLE
            
        return HabitatCandidate(
            x=sector['centroid'][0],
            y=sector['centroid'][1],
            z=self.current_position[2] if self.current_position else 0.0,
            quality=quality,
            score=total_score,
            flatness_score=flatness_score,
            shelter_score=shelter_score,
            accessibility_score=accessibility_score,
            area_score=area_score,
            timestamp=self.get_clock().now().nanoseconds / 1e9
        )
        
    def add_habitat_candidate(self, candidate: HabitatCandidate):
        """Add or update habitat candidate"""
        # Check if we already have a candidate near this location
        for i, existing in enumerate(self.habitat_candidates):
            distance = math.sqrt(
                (candidate.x - existing.x)**2 +
                (candidate.y - existing.y)**2
            )
            
            if distance < 2.0:  # Within 2 meters
                # Update if new one is better
                if candidate.score > existing.score:
                    self.habitat_candidates[i] = candidate
                return
                
        # Add new candidate
        self.habitat_candidates.append(candidate)
        
        # Keep only top candidates
        self.habitat_candidates.sort(key=lambda x: x.score, reverse=True)
        self.habitat_candidates = self.habitat_candidates[:50]
        
        if candidate.quality in [HabitatQuality.GOOD, HabitatQuality.EXCELLENT]:
            self.get_logger().info(
                f'🏠 Found {candidate.quality.name} habitat at ({candidate.x:.1f}, {candidate.y:.1f}) '
                f'Score: {candidate.score:.2f}'
            )
            
    def update_terrain_map(self, scan: LaserScan):
        """Update terrain map with current scan data"""
        if self.current_position is None:
            return
            
        for i, range_val in enumerate(scan.ranges):
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            if range_val < scan.range_min or range_val > scan.range_max:
                continue
                
            angle = scan.angle_min + i * scan.angle_increment
            x = self.current_position[0] + range_val * math.cos(angle)
            y = self.current_position[1] + range_val * math.sin(angle)
            
            # Convert to grid cell
            grid_x = int(x / self.grid_resolution)
            grid_y = int(y / self.grid_resolution)
            cell = (grid_x, grid_y)
            
            # This is where an obstacle was detected - mark it
            # (terrain analysis would be more sophisticated in practice)
            
    def publish_best_habitat(self):
        """Publish the best habitat location found"""
        if not self.habitat_candidates:
            return
            
        best = self.habitat_candidates[0]
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = best.x
        pose.pose.position.y = best.y
        pose.pose.position.z = best.z
        pose.pose.orientation.w = 1.0
        
        self.best_habitat_pub.publish(pose)
        
        # Publish candidates as JSON
        candidates_data = [{
            'x': c.x,
            'y': c.y,
            'score': c.score,
            'quality': c.quality.name
        } for c in self.habitat_candidates[:10]]
        
        msg = String()
        msg.data = json.dumps(candidates_data)
        self.candidates_pub.publish(msg)
        
    def publish_visualization(self):
        """Publish visualization markers"""
        marker_array = MarkerArray()
        
        # Delete old markers
        delete_marker = Marker()
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.header.frame_id = 'odom'
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, candidate in enumerate(self.habitat_candidates[:20]):
            # Habitat marker
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'odom'
            marker.ns = "habitats"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = candidate.x
            marker.pose.position.y = candidate.y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            
            # Size based on score
            size = 1.0 + candidate.score * 2.0
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = 0.2
            
            # Color based on quality
            marker.color = self.get_quality_color(candidate.quality)
            
            marker_array.markers.append(marker)
            
            # Label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "habitat_labels"
            text_marker.id = i + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = candidate.x
            text_marker.pose.position.y = candidate.y
            text_marker.pose.position.z = 1.5
            text_marker.scale.z = 0.4
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"#{i+1} {candidate.quality.name}\nScore: {candidate.score:.2f}"
            
            marker_array.markers.append(text_marker)
            
        self.marker_pub.publish(marker_array)
        
    def get_quality_color(self, quality: HabitatQuality) -> ColorRGBA:
        """Get color for habitat quality visualization"""
        color = ColorRGBA()
        color.a = 0.6
        
        if quality == HabitatQuality.EXCELLENT:
            color.r, color.g, color.b = 0.0, 1.0, 0.0  # Bright green
        elif quality == HabitatQuality.GOOD:
            color.r, color.g, color.b = 0.5, 1.0, 0.0  # Yellow-green
        elif quality == HabitatQuality.FAIR:
            color.r, color.g, color.b = 1.0, 1.0, 0.0  # Yellow
        elif quality == HabitatQuality.POOR:
            color.r, color.g, color.b = 1.0, 0.5, 0.0  # Orange
        else:
            color.r, color.g, color.b = 1.0, 0.0, 0.0  # Red
            
        return color
        
    def save_habitats(self):
        """Save habitat candidates to file"""
        if not self.habitat_candidates:
            return
            
        filename = f"habitats_{self.session_id}.json"
        filepath = os.path.join(self.save_directory, filename)
        
        data = {
            'session_id': self.session_id,
            'num_candidates': len(self.habitat_candidates),
            'candidates': [{
                'x': c.x,
                'y': c.y,
                'z': c.z,
                'quality': c.quality.name,
                'score': c.score,
                'flatness_score': c.flatness_score,
                'shelter_score': c.shelter_score,
                'accessibility_score': c.accessibility_score,
                'area_score': c.area_score,
                'timestamp': c.timestamp
            } for c in self.habitat_candidates]
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
            
        self.get_logger().info(f'💾 Saved {len(self.habitat_candidates)} habitat candidates')


def main(args=None):
    rclpy.init(args=args)
    node = HabitatFinderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_habitats()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

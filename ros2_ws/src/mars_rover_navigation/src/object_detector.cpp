#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <cmath>

class ObjectDetector : public rclcpp::Node
{
public:
  ObjectDetector() : Node("object_detector")
  {
    // Subscribe to lidar data
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10,
      std::bind(&ObjectDetector::scanCallback, this, std::placeholders::_1));
    
    // Publisher for detected objects/clusters
    objects_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/detected_objects", 10);
    
    // Parameters
    this->declare_parameter("min_cluster_size", 3);
    this->declare_parameter("max_cluster_distance", 0.5);
    this->declare_parameter("min_object_distance", 0.2);
    
    RCLCPP_INFO(this->get_logger(), "Object Detector Node Started");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::vector<std::vector<geometry_msgs::msg::Point>> clusters;
    std::vector<geometry_msgs::msg::Point> current_cluster;
    
    int min_cluster_size = this->get_parameter("min_cluster_size").as_int();
    double max_cluster_distance = this->get_parameter("max_cluster_distance").as_double();
    
    geometry_msgs::msg::Point last_point;
    bool first_point = true;
    
    // Convert laser scan to points and cluster them
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      float range = msg->ranges[i];
      
      // Skip invalid readings
      if (std::isnan(range) || std::isinf(range) || 
          range < msg->range_min || range > msg->range_max)
        continue;
      
      // Convert polar to Cartesian coordinates
      float angle = msg->angle_min + i * msg->angle_increment;
      geometry_msgs::msg::Point point;
      point.x = range * cos(angle);
      point.y = range * sin(angle);
      point.z = 0.0;
      
      if (first_point)
      {
        current_cluster.push_back(point);
        last_point = point;
        first_point = false;
        continue;
      }
      
      // Calculate distance to last point
      double distance = sqrt(
        pow(point.x - last_point.x, 2) + 
        pow(point.y - last_point.y, 2)
      );
      
      if (distance < max_cluster_distance)
      {
        // Add to current cluster
        current_cluster.push_back(point);
      }
      else
      {
        // Save cluster if it's large enough
        if (current_cluster.size() >= (size_t)min_cluster_size)
        {
          clusters.push_back(current_cluster);
        }
        // Start new cluster
        current_cluster.clear();
        current_cluster.push_back(point);
      }
      
      last_point = point;
    }
    
    // Don't forget the last cluster
    if (current_cluster.size() >= (size_t)min_cluster_size)
    {
      clusters.push_back(current_cluster);
    }
    
    // Publish detected objects as markers
    publishObjects(clusters, msg->header);
    
    RCLCPP_DEBUG(this->get_logger(), "Detected %zu objects", clusters.size());
  }
  
  void publishObjects(
    const std::vector<std::vector<geometry_msgs::msg::Point>>& clusters,
    const std_msgs::msg::Header& header)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    for (size_t i = 0; i < clusters.size(); ++i)
    {
      // Calculate centroid of cluster
      geometry_msgs::msg::Point centroid;
      centroid.x = 0; centroid.y = 0; centroid.z = 0;
      
      for (const auto& point : clusters[i])
      {
        centroid.x += point.x;
        centroid.y += point.y;
      }
      
      centroid.x /= clusters[i].size();
      centroid.y /= clusters[i].size();
      centroid.z = 0.3;  // Height for visualization
      
      // Create marker
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.ns = "detected_objects";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = centroid;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.6;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      
      marker_array.markers.push_back(marker);
    }
    
    objects_pub_->publish(marker_array);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objects_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetector>());
  rclcpp::shutdown();
  return 0;
}

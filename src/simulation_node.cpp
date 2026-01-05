#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <chrono>

class SimulationNode : public rclcpp::Node
{
public:
  SimulationNode()
  : Node("simulation_node"),
    publish_rate_(50.0),
    time_(0.0),
    vehicle_x_(0.0),
    vehicle_y_(0.0),
    vehicle_yaw_(0.0),
    vehicle_vx_(1.0),  // 1 m/s forward velocity
    path_radius_(10.0),  // 10m radius circular path
    path_center_x_(0.0),
    path_center_y_(0.0)
  {
    // Declare parameters
    declare_parameter("publish_rate", 50.0);
    declare_parameter("path_type", "circle");  // "circle" or "straight"
    declare_parameter("path_radius", 10.0);
    declare_parameter("vehicle_speed", 1.0);
    
    // Get parameters
    publish_rate_ = get_parameter("publish_rate").as_double();
    std::string path_type = get_parameter("path_type").as_string();
    path_radius_ = get_parameter("path_radius").as_double();
    vehicle_vx_ = get_parameter("vehicle_speed").as_double();
    
    // Create publishers
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/loader/imu", 10);
    lidar_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/loader/scan", 10);
    wheel_speed_pub_ = create_publisher<std_msgs::msg::Float64>("/loader/wheel_speed", 10);
    steering_rate_pub_ = create_publisher<std_msgs::msg::Float64>("/loader/steering_rate", 10);
    steering_feedback_pub_ = create_publisher<std_msgs::msg::Float64>("/loader/steering/feedback", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/loader/planning/path", 10);
    
    // Generate reference path
    generatePath(path_type);
    
    // Create timer
    double period = 1.0 / publish_rate_;
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period * 1000)),
      std::bind(&SimulationNode::timerCallback, this));
    
    // Publish path once at startup
    publishPath();
    
    RCLCPP_INFO(get_logger(), "Simulation node started");
    RCLCPP_INFO(get_logger(), "Path type: %s, Speed: %.2f m/s", path_type.c_str(), vehicle_vx_);
  }

private:
  void timerCallback()
  {
    double dt = 1.0 / publish_rate_;
    time_ += dt;
    
    // Update vehicle position based on path
    updateVehicleState(dt);
    
    // Publish all sensor data
    publishImu();
    publishLidar();
    publishWheelSpeed();
    publishSteeringRate();
    publishSteeringFeedback();
  }
  
  void generatePath(const std::string & path_type)
  {
    ref_path_.header.frame_id = "map";
    ref_path_.header.stamp = now();
    ref_path_.poses.clear();
    
    if (path_type == "circle") {
      // Generate circular path
      int num_points = 100;
      for (int i = 0; i <= num_points; ++i) {
        double theta = 2.0 * M_PI * i / num_points;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        pose.pose.position.x = path_center_x_ + path_radius_ * std::cos(theta);
        pose.pose.position.y = path_center_y_ + path_radius_ * std::sin(theta);
        pose.pose.position.z = 0.0;
        
        // Set orientation (tangent to circle)
        tf2::Quaternion q;
        q.setRPY(0, 0, theta + M_PI / 2.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        
        ref_path_.poses.push_back(pose);
      }
    } else {
      // Generate straight path
      double path_length = 50.0;
      int num_points = 100;
      for (int i = 0; i <= num_points; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        pose.pose.position.x = i * path_length / num_points;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        
        // Set orientation (straight ahead)
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        
        ref_path_.poses.push_back(pose);
      }
    }
    
    RCLCPP_INFO(get_logger(), "Generated path with %zu points", ref_path_.poses.size());
  }
  
  void updateVehicleState([[maybe_unused]] double dt)
  {
    if (ref_path_.poses.empty()) {
      return;
    }
    
    // Simple vehicle dynamics: move along path
    // For circular path: vehicle follows the circle
    if (ref_path_.poses.size() > 50) {  // Assume circular path
      double circumference = 2.0 * M_PI * path_radius_;
      double distance = std::fmod(vehicle_vx_ * time_, circumference);
      double theta = 2.0 * M_PI * distance / circumference;
      
      vehicle_x_ = path_center_x_ + path_radius_ * std::cos(theta);
      vehicle_y_ = path_center_y_ + path_radius_ * std::sin(theta);
      vehicle_yaw_ = theta + M_PI / 2.0;
    } else {
      // Straight path
      vehicle_x_ = vehicle_vx_ * time_;
      vehicle_y_ = 0.0;
      vehicle_yaw_ = 0.0;
    }
  }
  
  void publishImu()
  {
    auto msg = std::make_shared<sensor_msgs::msg::Imu>();
    msg->header.frame_id = "base_link";
    msg->header.stamp = now();
    
    // Set orientation from yaw
    tf2::Quaternion q;
    q.setRPY(0, 0, vehicle_yaw_);
    msg->orientation.x = q.x();
    msg->orientation.y = q.y();
    msg->orientation.z = q.z();
    msg->orientation.w = q.w();
    
    // Set angular velocity (for circular path)
    if (ref_path_.poses.size() > 50) {
      msg->angular_velocity.z = vehicle_vx_ / path_radius_;  // omega = v / r
    } else {
      msg->angular_velocity.z = 0.0;
    }
    
    // Set linear acceleration (centripetal for circular path)
    if (ref_path_.poses.size() > 50) {
      double centripetal_accel = vehicle_vx_ * vehicle_vx_ / path_radius_;
      msg->linear_acceleration.x = -centripetal_accel * std::cos(vehicle_yaw_);
      msg->linear_acceleration.y = -centripetal_accel * std::sin(vehicle_yaw_);
    }
    
    imu_pub_->publish(*msg);
  }
  
  void publishLidar()
  {
    auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    msg->header.frame_id = "laser";
    msg->header.stamp = now();
    
    msg->angle_min = -M_PI;
    msg->angle_max = M_PI;
    msg->angle_increment = M_PI / 180.0;  // 1 degree
    msg->time_increment = 0.0;
    msg->scan_time = 1.0 / publish_rate_;
    msg->range_min = 0.1;
    msg->range_max = 10.0;
    
    // Generate fake LiDAR data (all obstacles far away)
    int num_readings = static_cast<int>((msg->angle_max - msg->angle_min) / msg->angle_increment);
    msg->ranges.resize(num_readings, msg->range_max);
    msg->intensities.resize(num_readings, 1.0);
    
    lidar_pub_->publish(*msg);
  }
  
  void publishWheelSpeed()
  {
    auto msg = std::make_shared<std_msgs::msg::Float64>();
    msg->data = vehicle_vx_;
    wheel_speed_pub_->publish(*msg);
  }
  
  void publishSteeringRate()
  {
    auto msg = std::make_shared<std_msgs::msg::Float64>();
    // Simulate steering rate based on path curvature
    if (ref_path_.poses.size() > 50) {
      msg->data = vehicle_vx_ / path_radius_;  // Approximate steering rate
    } else {
      msg->data = 0.0;
    }
    steering_rate_pub_->publish(*msg);
  }
  
  void publishSteeringFeedback()
  {
    auto msg = std::make_shared<std_msgs::msg::Float64>();
    // Simulate steering feedback (with some delay/noise)
    // For circular path: steering angle = atan(L * curvature)
    double wheelbase = 2.5;  // Assume 2.5m wheelbase
    if (ref_path_.poses.size() > 50) {
      double curvature = 1.0 / path_radius_;
      msg->data = std::atan(wheelbase * curvature);
    } else {
      msg->data = 0.0;
    }
    steering_feedback_pub_->publish(*msg);
  }
  
  void publishPath()
  {
    ref_path_.header.stamp = now();
    path_pub_->publish(ref_path_);
  }
  
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wheel_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_rate_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_feedback_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  double publish_rate_;
  double time_;
  double vehicle_x_, vehicle_y_, vehicle_yaw_;
  double vehicle_vx_;
  double path_radius_;
  double path_center_x_, path_center_y_;
  
  nav_msgs::msg::Path ref_path_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimulationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


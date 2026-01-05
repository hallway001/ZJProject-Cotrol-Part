#include "loader_control/data_receiver.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace loader_control
{

DataReceiver::DataReceiver(const std::string & node_name)
: Node(node_name),
  vehicle_state_(),
  use_synchronization_(true),
  sync_queue_size_(10),
  sync_time_window_(0.1),
  use_imu_yaw_(false),
  last_imu_time_(0.0),
  last_yaw_(0.0)
{
  // Declare parameters
  declare_parameter("use_synchronization", true);
  declare_parameter("sync_queue_size", 10);
  declare_parameter("sync_time_window", 0.1);
  declare_parameter("sensors.imu.topic", "/loader/imu");
  declare_parameter("sensors.lidar.topic", "/loader/scan");
  declare_parameter("sensors.wheel_speed.topic", "/loader/wheel_speed");
  declare_parameter("sensors.steering_rate.topic", "/loader/steering_rate");
  declare_parameter("sensors.steering_feedback.topic", "/loader/steering/feedback");
  declare_parameter("sensors.path.topic", "/loader/planning/path");
  
  // Get parameters
  use_synchronization_ = get_parameter("use_synchronization").as_bool();
  sync_queue_size_ = get_parameter("sync_queue_size").as_int();
  sync_time_window_ = get_parameter("sync_time_window").as_double();
  
  std::string imu_topic = get_parameter("sensors.imu.topic").as_string();
  std::string lidar_topic = get_parameter("sensors.lidar.topic").as_string();
  std::string wheel_speed_topic = get_parameter("sensors.wheel_speed.topic").as_string();
  std::string steering_rate_topic = get_parameter("sensors.steering_rate.topic").as_string();
  std::string steering_fb_topic = get_parameter("sensors.steering_feedback.topic").as_string();
  std::string path_topic = get_parameter("sensors.path.topic").as_string();
  
  // Create subscribers
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, 10,
    std::bind(&DataReceiver::imuCallback, this, std::placeholders::_1));
  
  lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    lidar_topic, 10,
    std::bind(&DataReceiver::lidarCallback, this, std::placeholders::_1));
  
  wheel_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
    wheel_speed_topic, 10,
    std::bind(&DataReceiver::wheelSpeedCallback, this, std::placeholders::_1));
  
  steering_rate_sub_ = create_subscription<std_msgs::msg::Float64>(
    steering_rate_topic, 10,
    std::bind(&DataReceiver::steeringRateCallback, this, std::placeholders::_1));
  
  steering_feedback_sub_ = create_subscription<std_msgs::msg::Float64>(
    steering_fb_topic, 10,
    std::bind(&DataReceiver::steeringFeedbackCallback, this, std::placeholders::_1));
  
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    path_topic, 10,
    std::bind(&DataReceiver::pathCallback, this, std::placeholders::_1));
  
  // Initialize message filters if using synchronization
  if (use_synchronization_) {
    imu_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(
      shared_from_this(), imu_topic);
    wheel_speed_filter_sub_ = std::make_shared<message_filters::Subscriber<std_msgs::msg::Float64>>(
      shared_from_this(), wheel_speed_topic);
    
    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_size_), *imu_filter_sub_, *wheel_speed_filter_sub_);
    synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_time_window_));
    synchronizer_->registerCallback(
      std::bind(&DataReceiver::synchronizedCallback, this,
        std::placeholders::_1, std::placeholders::_2));
  }
  
  vehicle_state_.stamp = now();
}

VehicleState DataReceiver::getVehicleState() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return vehicle_state_;
}

bool DataReceiver::isStateValid() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return vehicle_state_.is_valid;
}

void DataReceiver::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  updateStateFromImu(msg);
  vehicle_state_.stamp = msg->header.stamp;
}

void DataReceiver::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // For now, just store LiDAR data (can be used for obstacle avoidance later)
  // No need to lock since we're not updating critical state here
}

void DataReceiver::wheelSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.wheel_speed = msg->data;
  vehicle_state_.vx = msg->data;  // Assuming forward velocity
}

void DataReceiver::steeringRateCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.steering_rate = msg->data;
}

void DataReceiver::steeringFeedbackCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.delta_fb = msg->data;
  vehicle_state_.updateSteeringError();
}

void DataReceiver::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.ref_path = *msg;
  findClosestPathPoint(vehicle_state_);
  vehicle_state_.path_curvature = calculatePathCurvature(vehicle_state_.ref_path, vehicle_state_.closest_path_index);
}

void DataReceiver::synchronizedCallback(
  const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg,
  const std_msgs::msg::Float64::ConstSharedPtr & wheel_speed_msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  updateStateFromImu(std::const_pointer_cast<sensor_msgs::msg::Imu>(imu_msg));
  vehicle_state_.wheel_speed = wheel_speed_msg->data;
  vehicle_state_.vx = wheel_speed_msg->data;
  vehicle_state_.stamp = imu_msg->header.stamp;
  vehicle_state_.is_valid = true;
}

void DataReceiver::updateStateFromImu(const sensor_msgs::msg::Imu::SharedPtr & imu_msg)
{
  vehicle_state_.imu = *imu_msg;
  vehicle_state_.omega = imu_msg->angular_velocity.z;
  
  // Extract yaw from IMU quaternion (simplified, assuming flat ground)
  double qx = imu_msg->orientation.x;
  double qy = imu_msg->orientation.y;
  double qz = imu_msg->orientation.z;
  double qw = imu_msg->orientation.w;
  
  // Convert quaternion to yaw
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  vehicle_state_.yaw = std::atan2(siny_cosp, cosy_cosp);
  
  // Simple dead reckoning (can be improved with EKF)
  if (use_imu_yaw_ && last_imu_time_ > 0.0) {
    double dt = (rclcpp::Time(imu_msg->header.stamp) - rclcpp::Time(vehicle_state_.stamp)).seconds();
    if (dt > 0.0 && dt < 1.0) {
      // Integrate velocity to update position
      vehicle_state_.x += vehicle_state_.vx * std::cos(vehicle_state_.yaw) * dt;
      vehicle_state_.y += vehicle_state_.vx * std::sin(vehicle_state_.yaw) * dt;
    }
  }
  
  last_imu_time_ = rclcpp::Time(imu_msg->header.stamp).seconds();
}

void DataReceiver::findClosestPathPoint(VehicleState & state)
{
  if (state.ref_path.poses.empty()) {
    state.closest_path_index = 0;
    return;
  }
  
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_idx = 0;
  
  for (size_t i = 0; i < state.ref_path.poses.size(); ++i) {
    const auto & pose = state.ref_path.poses[i].pose.position;
    double dx = pose.x - state.x;
    double dy = pose.y - state.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  
  state.closest_path_index = closest_idx;
}

double DataReceiver::calculatePathCurvature(
  const nav_msgs::msg::Path & path, size_t index)
{
  if (path.poses.size() < 3 || index == 0 || index >= path.poses.size() - 1) {
    return 0.0;
  }
  
  // Use three points to calculate curvature
  const auto & p0 = path.poses[index - 1].pose.position;
  const auto & p1 = path.poses[index].pose.position;
  const auto & p2 = path.poses[index + 1].pose.position;
  
  // Calculate curvature using cross product method
  double dx1 = p1.x - p0.x;
  double dy1 = p1.y - p0.y;
  double dx2 = p2.x - p1.x;
  double dy2 = p2.y - p1.y;
  
  double ds1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
  double ds2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
  
  if (ds1 < 1e-6 || ds2 < 1e-6) {
    return 0.0;
  }
  
  // Normalize
  dx1 /= ds1;
  dy1 /= ds1;
  dx2 /= ds2;
  dy2 /= ds2;
  
  // Curvature = cross product / (ds1 * ds2)
  double cross = dx1 * dy2 - dy1 * dx2;
  double curvature = std::abs(cross) / ((ds1 + ds2) / 2.0);
  
  return curvature;
}

}  // namespace loader_control

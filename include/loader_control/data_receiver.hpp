#ifndef LOADER_CONTROL__DATA_RECEIVER_HPP_
#define LOADER_CONTROL__DATA_RECEIVER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "loader_control/vehicle_state.hpp"

namespace loader_control
{

/**
 * @brief Sensor data receiver and synchronizer
 * Handles subscription and time synchronization of multiple sensor topics
 */
class DataReceiver : public rclcpp::Node
{
public:
  explicit DataReceiver(const std::string & node_name = "data_receiver");
  
  /**
   * @brief Get current vehicle state
   */
  VehicleState getVehicleState() const;
  
  /**
   * @brief Check if vehicle state is valid
   */
  bool isStateValid() const;

protected:
  // Callbacks for individual sensors
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void wheelSpeedCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void steeringRateCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void steeringFeedbackCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  
  // Synchronized callback (for time synchronization)
  void synchronizedCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg,
    const std_msgs::msg::Float64::ConstSharedPtr & wheel_speed_msg);
  
  // Helper functions
  void updateStateFromImu(const sensor_msgs::msg::Imu::SharedPtr & imu_msg);
  void findClosestPathPoint(VehicleState & state);
  double calculatePathCurvature(const nav_msgs::msg::Path & path, size_t index);
  
private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_rate_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_feedback_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  
  // Message filters for time synchronization
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_filter_sub_;
  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Float64>> wheel_speed_filter_sub_;
  
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Imu,
    std_msgs::msg::Float64> SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  
  // Vehicle state (protected by mutex)
  mutable std::mutex state_mutex_;
  VehicleState vehicle_state_;
  
  // Configuration parameters
  bool use_synchronization_;
  double sync_queue_size_;
  double sync_time_window_;
  
  // State estimation parameters
  bool use_imu_yaw_;
  double last_imu_time_;
  double last_yaw_;
};

}  // namespace loader_control

#endif  // LOADER_CONTROL__DATA_RECEIVER_HPP_


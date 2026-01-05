#ifndef LOADER_CONTROL__VEHICLE_STATE_HPP_
#define LOADER_CONTROL__VEHICLE_STATE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace loader_control
{

/**
 * @brief Vehicle state structure containing position, velocity, and control references
 */
struct VehicleState
{
  // Pose (position and orientation)
  double x = 0.0;              // Position x (m)
  double y = 0.0;              // Position y (m)
  double yaw = 0.0;            // Orientation yaw angle (rad)
  
  // Velocity
  double vx = 0.0;             // Linear velocity x (m/s)
  double vy = 0.0;             // Linear velocity y (m/s)
  double omega = 0.0;          // Angular velocity (rad/s)
  
  // IMU data
  sensor_msgs::msg::Imu imu;
  
  // Control commands and feedback
  double delta_cmd = 0.0;      // Controller output steering angle command (rad)
  double delta_fb = 0.0;       // Actual feedback steering angle (rad)
  double steering_error = 0.0; // Steering error = delta_cmd - delta_fb (rad)
  
  // Wheel speed
  double wheel_speed = 0.0;    // Wheel speed (m/s)
  double steering_rate = 0.0;  // Steering angular rate (rad/s)
  
  // Reference path
  nav_msgs::msg::Path ref_path; // Reference trajectory path
  
  // Current path tracking info
  double lookahead_distance = 2.0; // Current lookahead distance (m)
  double path_curvature = 0.0;     // Current path curvature (1/m)
  int closest_path_index = 0;      // Index of closest point on path
  
  // Timestamp
  rclcpp::Time stamp;
  
  // Validity flag
  bool is_valid = false;
  
  /**
   * @brief Calculate steering error
   */
  void updateSteeringError()
  {
    steering_error = delta_cmd - delta_fb;
  }
  
  /**
   * @brief Reset state
   */
  void reset()
  {
    x = y = yaw = 0.0;
    vx = vy = omega = 0.0;
    delta_cmd = delta_fb = steering_error = 0.0;
    wheel_speed = steering_rate = 0.0;
    lookahead_distance = 2.0;
    path_curvature = 0.0;
    closest_path_index = 0;
    is_valid = false;
  }
};

}  // namespace loader_control

#endif  // LOADER_CONTROL__VEHICLE_STATE_HPP_


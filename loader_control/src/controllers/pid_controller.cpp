#include "loader_control/controllers/pid_controller.hpp"
#include "loader_control/utils/trajectory_interpolator.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace loader_control {

void PIDController::configure(const rclcpp::Node::SharedPtr & node) {
  // 获取参数
  node->declare_parameter("controller.pid.lat.kp", 1.2);
  node->declare_parameter("controller.pid.lat.ki", 0.1);
  node->declare_parameter("controller.pid.lat.kd", 0.3);
  node->declare_parameter("controller.pid.heading.kp", 0.8);
  node->declare_parameter("controller.pid.heading.ki", 0.05);
  node->declare_parameter("controller.pid.heading.kd", 0.2);
  node->declare_parameter("max_steering", 0.6);
  node->declare_parameter("max_throttle", 0.8);
  node->declare_parameter("wheelbase", 2.8);

  pid_params_.kp_lat = node->get_parameter("controller.pid.lat.kp").as_double();
  pid_params_.ki_lat = node->get_parameter("controller.pid.lat.ki").as_double();
  pid_params_.kd_lat = node->get_parameter("controller.pid.lat.kd").as_double();
  pid_params_.kp_heading = node->get_parameter("controller.pid.heading.kp").as_double();
  pid_params_.ki_heading = node->get_parameter("controller.pid.heading.ki").as_double();
  pid_params_.kd_heading = node->get_parameter("controller.pid.heading.kd").as_double();
  max_steering_ = node->get_parameter("max_steering").as_double();
  max_throttle_ = node->get_parameter("max_throttle").as_double();
  wheelbase_ = node->get_parameter("wheelbase").as_double();
}

ControllerOutput PIDController::compute(const ControllerInput & input) {
  ControllerOutput output;

  // 计算横向误差和航向误差
  double error_lat = computeLateralError(input);
  double error_heading = computeHeadingError(input);

  // PID控制 - 横向误差
  pid_state_.error_lat_integral += error_lat;
  double error_lat_derivative = error_lat - pid_state_.last_error_lat;
  double steering_lat = pid_params_.kp_lat * error_lat +
                       pid_params_.ki_lat * pid_state_.error_lat_integral +
                       pid_params_.kd_lat * error_lat_derivative;
  pid_state_.last_error_lat = error_lat;

  // PID控制 - 航向误差
  pid_state_.error_heading_integral += error_heading;
  double error_heading_derivative = error_heading - pid_state_.last_error_heading;
  double steering_heading = pid_params_.kp_heading * error_heading +
                           pid_params_.ki_heading * pid_state_.error_heading_integral +
                           pid_params_.kd_heading * error_heading_derivative;
  pid_state_.last_error_heading = error_heading;

  // 组合转向指令
  output.steering_cmd = steering_lat + steering_heading;
  output.steering_cmd = std::max(-max_steering_, std::min(max_steering_, output.steering_cmd));

  // 简单的速度控制（可根据需要调整）
  double speed_error = input.reference_path.poses.empty() ? 0.0 : 
                      std::abs(error_lat) * 0.5;  // 误差越大，速度越小
  output.throttle_cmd = std::max(-max_throttle_, std::min(max_throttle_, 0.5 - speed_error));

  return output;
}

void PIDController::reset() {
  pid_state_.error_lat_integral = 0.0;
  pid_state_.error_heading_integral = 0.0;
  pid_state_.last_error_lat = 0.0;
  pid_state_.last_error_heading = 0.0;
}

double PIDController::computeLateralError(const ControllerInput & input) {
  if (input.reference_path.poses.empty() || input.closest_idx >= input.reference_path.poses.size()) {
    return 0.0;
  }

  const auto & ref_pose = input.reference_path.poses[input.closest_idx].pose;
  
  // 计算当前点到参考路径的距离（简化版，使用最近点）
  double dx = ref_pose.position.x - input.current_pose.pose.position.x;
  double dy = ref_pose.position.y - input.current_pose.pose.position.y;
  
  // 获取当前朝向角度
  tf2::Quaternion q;
  tf2::fromMsg(input.current_pose.pose.orientation, q);
  double yaw = tf2::getYaw(q);
  
  // 计算横向误差
  double lateral_error = -dx * sin(yaw) + dy * cos(yaw);
  
  return lateral_error;
}

double PIDController::computeHeadingError(const ControllerInput & input) {
  if (input.reference_path.poses.empty() || input.closest_idx >= input.reference_path.poses.size()) {
    return 0.0;
  }

  const auto & ref_pose = input.reference_path.poses[input.closest_idx].pose;
  
  // 获取当前航向
  tf2::Quaternion q_current;
  tf2::fromMsg(input.current_pose.pose.orientation, q_current);
  double yaw_current = tf2::getYaw(q_current);
  
  // 获取参考航向
  tf2::Quaternion q_ref;
  tf2::fromMsg(ref_pose.orientation, q_ref);
  double yaw_ref = tf2::getYaw(q_ref);
  
  // 计算航向误差（归一化到[-pi, pi]）
  double error = yaw_ref - yaw_current;
  while (error > M_PI) error -= 2.0 * M_PI;
  while (error < -M_PI) error += 2.0 * M_PI;
  
  return error;
}

} // namespace loader_control


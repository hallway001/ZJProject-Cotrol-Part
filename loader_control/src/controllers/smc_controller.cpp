#include "loader_control/controllers/smc_controller.hpp"
#include "loader_control/utils/trajectory_interpolator.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace loader_control {

void SMCController::configure(const rclcpp::Node::SharedPtr & node) {
  node->declare_parameter("controller.smc.k_lat", 1.0);
  node->declare_parameter("controller.smc.k_heading", 1.0);
  node->declare_parameter("controller.smc.lambda_lat", 0.5);
  node->declare_parameter("controller.smc.lambda_heading", 0.5);
  node->declare_parameter("max_steering", 0.6);
  node->declare_parameter("max_throttle", 0.8);
  node->declare_parameter("wheelbase", 2.8);

  k_lat_ = node->get_parameter("controller.smc.k_lat").as_double();
  k_heading_ = node->get_parameter("controller.smc.k_heading").as_double();
  lambda_lat_ = node->get_parameter("controller.smc.lambda_lat").as_double();
  lambda_heading_ = node->get_parameter("controller.smc.lambda_heading").as_double();
  max_steering_ = node->get_parameter("max_steering").as_double();
  max_throttle_ = node->get_parameter("max_throttle").as_double();
  wheelbase_ = node->get_parameter("wheelbase").as_double();
}

ControllerOutput SMCController::compute(const ControllerInput & input) {
  ControllerOutput output;

  // 计算横向误差和航向误差
  double error_lat = computeLateralError(input);
  double error_heading = computeHeadingError(input);

  // 滑模面
  double s_lat = error_lat + lambda_lat_ * error_lat;  // 简化版，实际应包含误差导数
  double s_heading = error_heading + lambda_heading_ * error_heading;

  // 滑模控制律（简化版）
  double u_lat = -k_lat_ * std::tanh(s_lat / 0.1);  // 使用tanh代替sign函数以减小抖振
  double u_heading = -k_heading_ * std::tanh(s_heading / 0.1);

  // 组合控制指令
  output.steering_cmd = u_lat + u_heading;
  output.steering_cmd = std::max(-max_steering_, std::min(max_steering_, output.steering_cmd));

  // 速度控制
  double speed_factor = 1.0 - std::abs(error_lat) * 0.3;
  output.throttle_cmd = max_throttle_ * speed_factor;
  output.throttle_cmd = std::max(-max_throttle_, std::min(max_throttle_, output.throttle_cmd));

  return output;
}

void SMCController::reset() {
  // SMC无状态，无需重置
}

double SMCController::computeLateralError(const ControllerInput & input) {
  if (input.reference_path.poses.empty() || input.closest_idx >= input.reference_path.poses.size()) {
    return 0.0;
  }

  const auto & ref_pose = input.reference_path.poses[input.closest_idx].pose;
  
  double dx = ref_pose.position.x - input.current_pose.pose.position.x;
  double dy = ref_pose.position.y - input.current_pose.pose.position.y;
  
  tf2::Quaternion q;
  tf2::fromMsg(input.current_pose.pose.orientation, q);
  double yaw = tf2::getYaw(q);
  
  double lateral_error = -dx * sin(yaw) + dy * cos(yaw);
  
  return lateral_error;
}

double SMCController::computeHeadingError(const ControllerInput & input) {
  if (input.reference_path.poses.empty() || input.closest_idx >= input.reference_path.poses.size()) {
    return 0.0;
  }

  const auto & ref_pose = input.reference_path.poses[input.closest_idx].pose;
  
  tf2::Quaternion q_current;
  tf2::fromMsg(input.current_pose.pose.orientation, q_current);
  double yaw_current = tf2::getYaw(q_current);
  
  tf2::Quaternion q_ref;
  tf2::fromMsg(ref_pose.orientation, q_ref);
  double yaw_ref = tf2::getYaw(q_ref);
  
  double error = yaw_ref - yaw_current;
  while (error > M_PI) error -= 2.0 * M_PI;
  while (error < -M_PI) error += 2.0 * M_PI;
  
  return error;
}

} // namespace loader_control


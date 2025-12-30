#include "loader_control/controllers/mpc_controller.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace loader_control {

void MPCController::configure(const rclcpp::Node::SharedPtr & node) {
  node->declare_parameter("controller.mpc.horizon", 10);
  node->declare_parameter("controller.mpc.dt", 0.05);
  node->declare_parameter("controller.mpc.Q", std::vector<double>{1.0, 0.5, 0.2});
  node->declare_parameter("controller.mpc.R", std::vector<double>{0.01, 0.1});
  node->declare_parameter("max_steering", 0.6);
  node->declare_parameter("max_throttle", 0.8);
  node->declare_parameter("wheelbase", 2.8);

  horizon_ = node->get_parameter("controller.mpc.horizon").as_int();
  dt_ = node->get_parameter("controller.mpc.dt").as_double();
  
  auto Q_vec = node->get_parameter("controller.mpc.Q").as_double_array();
  if (Q_vec.size() >= 3) {
    Q_ = Eigen::Vector3d(Q_vec[0], Q_vec[1], Q_vec[2]);
  }
  
  auto R_vec = node->get_parameter("controller.mpc.R").as_double_array();
  if (R_vec.size() >= 2) {
    R_ = Eigen::Vector2d(R_vec[0], R_vec[1]);
  }
  
  max_steering_ = node->get_parameter("max_steering").as_double();
  max_throttle_ = node->get_parameter("max_throttle").as_double();
  wheelbase_ = node->get_parameter("wheelbase").as_double();
}

ControllerOutput MPCController::compute(const ControllerInput & input) {
  ControllerOutput output;
  
  // 调用MPC求解器
  Eigen::Vector2d u = solveMPC(input);
  
  output.throttle_cmd = std::max(-max_throttle_, std::min(max_throttle_, u(0)));
  output.steering_cmd = std::max(-max_steering_, std::min(max_steering_, u(1)));
  
  return output;
}

void MPCController::reset() {
  // MPC无状态，无需重置
}

Eigen::Vector2d MPCController::solveMPC(const ControllerInput & input) {
  // 简化的MPC实现（实际应使用qpOASES或其他QP求解器）
  // 这里提供一个基础的实现框架
  
  if (input.reference_path.poses.empty()) {
    return Eigen::Vector2d(0.0, 0.0);
  }

  // 获取当前状态
  tf2::Quaternion q;
  tf2::fromMsg(input.current_pose.pose.orientation, q);
  double yaw = tf2::getYaw(q);
  
  double x = input.current_pose.pose.position.x;
  double y = input.current_pose.pose.position.y;
  double v = input.current_speed;
  double delta = input.current_steering;

  // 获取参考轨迹上的目标点
  size_t target_idx = input.closest_idx;
  if (target_idx >= input.reference_path.poses.size()) {
    target_idx = input.reference_path.poses.size() - 1;
  }
  
  const auto & ref_pose = input.reference_path.poses[target_idx].pose;
  tf2::Quaternion q_ref;
  tf2::fromMsg(ref_pose.orientation, q_ref);
  double yaw_ref = tf2::getYaw(q_ref);
  
  double x_ref = ref_pose.position.x;
  double y_ref = ref_pose.position.y;

  // 计算误差
  double e_x = x - x_ref;
  double e_y = y - y_ref;
  double e_yaw = yaw - yaw_ref;
  
  // 归一化航向误差
  while (e_yaw > M_PI) e_yaw -= 2.0 * M_PI;
  while (e_yaw < -M_PI) e_yaw += 2.0 * M_PI;

  // 简化的控制律（可替换为完整的QP求解）
  // 这里使用类似LQR的方法作为简化实现
  double steering = -0.5 * e_yaw - 0.3 * e_y;
  double throttle = 0.5 * (1.0 - std::abs(e_y) * 0.5);  // 误差越大，速度越小

  return Eigen::Vector2d(throttle, steering);
}

} // namespace loader_control


#include "loader_control/utils/command_modulator.hpp"
#include <algorithm>
#include <cmath>

namespace loader_control {

CommandModulator::CommandModulator(const rclcpp::Node::SharedPtr & node) {
  node->declare_parameter("max_steering", 0.6);
  node->declare_parameter("max_throttle", 0.8);
  node->declare_parameter("steering_rate_limit", 0.1);
  node->declare_parameter("throttle_alpha", 0.8);
  node->declare_parameter("steering_alpha", 0.9);
  node->declare_parameter("enable_delay_compensation", true);
  node->declare_parameter("tau_v", 0.8);

  max_steering_ = node->get_parameter("max_steering").as_double();
  max_throttle_ = node->get_parameter("max_throttle").as_double();
  steering_rate_limit_ = node->get_parameter("steering_rate_limit").as_double();
  throttle_alpha_ = node->get_parameter("throttle_alpha").as_double();
  steering_alpha_ = node->get_parameter("steering_alpha").as_double();
  enable_delay_comp_ = node->get_parameter("enable_delay_compensation").as_bool();
  tau_v_ = node->get_parameter("tau_v").as_double();
}

loader_control_interfaces::msg::ActuatorCmd CommandModulator::modulate(
  const loader_control_interfaces::msg::RawControlCmd & raw,
  double dt) {
  
  loader_control_interfaces::msg::ActuatorCmd cmd;
  
  // 限制幅值
  double throttle_clamped = std::max(-max_throttle_, std::min(max_throttle_, raw.throttle));
  double steering_clamped = std::max(-max_steering_, std::min(max_steering_, raw.steering));
  
  // 速率限制 - 转向
  double steering_rate = (steering_clamped - last_steering_) / std::max(dt, 1e-6);
  if (std::abs(steering_rate) > steering_rate_limit_) {
    steering_clamped = last_steering_ + std::copysign(steering_rate_limit_ * dt, steering_rate);
  }
  
  // 一阶低通滤波
  last_throttle_filtered_ = throttle_alpha_ * last_throttle_filtered_ + 
                            (1.0 - throttle_alpha_) * throttle_clamped;
  last_steering_filtered_ = steering_alpha_ * last_steering_filtered_ + 
                            (1.0 - steering_alpha_) * steering_clamped;
  
  // 延迟补偿（简化实现）
  if (enable_delay_comp_) {
    // 对油门进行延迟补偿预测
    double throttle_comp = last_throttle_filtered_ + 
                          (throttle_clamped - last_throttle_filtered_) * (1.0 - std::exp(-dt / tau_v_));
    last_throttle_filtered_ = throttle_comp;
  }
  
  cmd.throttle = last_throttle_filtered_;
  cmd.steering = last_steering_filtered_;
  
  // 更新状态
  last_steering_ = steering_clamped;
  
  return cmd;
}

} // namespace loader_control


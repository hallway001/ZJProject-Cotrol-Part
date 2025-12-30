#pragma once
#include "rclcpp/rclcpp.hpp"
#include "loader_control_interfaces/msg/raw_control_cmd.hpp"
#include "loader_control_interfaces/msg/actuator_cmd.hpp"
#include <memory>

namespace loader_control {

class CommandModulator {
public:
  explicit CommandModulator(const rclcpp::Node::SharedPtr & node);
  
  loader_control_interfaces::msg::ActuatorCmd modulate(
    const loader_control_interfaces::msg::RawControlCmd & raw,
    double dt);

private:
  // 参数
  double max_steering_{0.6};
  double max_throttle_{0.8};
  double steering_rate_limit_{0.1};
  double throttle_alpha_{0.8};
  double steering_alpha_{0.9};
  bool enable_delay_comp_{true};
  double tau_v_{0.8};

  // 状态
  double last_steering_{0.0};
  double last_throttle_filtered_{0.0};
  double last_steering_filtered_{0.0};
};

} // namespace loader_control


#pragma once
#include "rclcpp/rclcpp.hpp"
#include "loader_control_interfaces/msg/vehicle_state.hpp"
#include "loader_control_interfaces/msg/raw_control_cmd.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace loader_control {

struct ControllerInput {
  geometry_msgs::msg::PoseStamped current_pose;
  double current_speed;
  double current_steering;
  nav_msgs::msg::Path reference_path;
  size_t closest_idx;
};

struct ControllerOutput {
  double throttle_cmd;   // [-1.0, 1.0] → forward/backward
  double steering_cmd;   // [-max_steering, max_steering]
};

class BaseController {
public:
  virtual ~BaseController() = default;
  virtual void configure(const rclcpp::Node::SharedPtr & node) = 0;
  virtual ControllerOutput compute(const ControllerInput & input) = 0;
  virtual void reset() {}
};

} // namespace loader_control


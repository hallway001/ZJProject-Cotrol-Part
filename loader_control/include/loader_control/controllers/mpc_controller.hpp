#pragma once
#include "loader_control/controllers/base_controller.hpp"
#include <Eigen/Dense>
#include <memory>

namespace loader_control {

class MPCController : public BaseController {
public:
  MPCController() = default;
  ~MPCController() override = default;

  void configure(const rclcpp::Node::SharedPtr & node) override;
  ControllerOutput compute(const ControllerInput & input) override;
  void reset() override;

private:
  int horizon_{10};
  double dt_{0.05};
  Eigen::Vector3d Q_;  // [x_error, y_error, heading_error]
  Eigen::Vector2d R_;  // [throttle, steering]
  double max_steering_{0.6};
  double max_throttle_{0.8};
  double wheelbase_{2.8};

  // Simplified MPC solver (placeholder for qpOASES integration)
  Eigen::Vector2d solveMPC(const ControllerInput & input);
};

} // namespace loader_control


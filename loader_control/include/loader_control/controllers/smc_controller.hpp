#pragma once
#include "loader_control/controllers/base_controller.hpp"
#include <memory>

namespace loader_control {

class SMCController : public BaseController {
public:
  SMCController() = default;
  ~SMCController() override = default;

  void configure(const rclcpp::Node::SharedPtr & node) override;
  ControllerOutput compute(const ControllerInput & input) override;
  void reset() override;

private:
  double k_lat_{1.0};
  double k_heading_{1.0};
  double lambda_lat_{0.5};
  double lambda_heading_{0.5};
  double max_steering_{0.6};
  double max_throttle_{0.8};
  double wheelbase_{2.8};

  double computeLateralError(const ControllerInput & input);
  double computeHeadingError(const ControllerInput & input);
};

} // namespace loader_control


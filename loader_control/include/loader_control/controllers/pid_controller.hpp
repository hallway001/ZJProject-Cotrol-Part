#pragma once
#include "loader_control/controllers/base_controller.hpp"
#include <memory>

namespace loader_control {

class PIDController : public BaseController {
public:
  PIDController() = default;
  ~PIDController() override = default;

  void configure(const rclcpp::Node::SharedPtr & node) override;
  ControllerOutput compute(const ControllerInput & input) override;
  void reset() override;

private:
  struct PIDParams {
    double kp_lat, ki_lat, kd_lat;
    double kp_heading, ki_heading, kd_heading;
  } pid_params_;

  struct PIDState {
    double error_lat_integral{0.0};
    double error_heading_integral{0.0};
    double last_error_lat{0.0};
    double last_error_heading{0.0};
  } pid_state_;

  double max_steering_{0.6};
  double max_throttle_{0.8};
  double wheelbase_{2.8};

  double computeLateralError(const ControllerInput & input);
  double computeHeadingError(const ControllerInput & input);
};

} // namespace loader_control


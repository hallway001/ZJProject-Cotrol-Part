#pragma once
#include "loader_control_interfaces/msg/vehicle_state.hpp"
#include <memory>

namespace loader_control {

class UKFEstimator {
public:
  UKFEstimator() = default;
  
  void configure(double wheelbase, double tau_v, double tau_delta,
                const std::vector<double> & process_noise,
                const std::vector<double> & measurement_noise);
  
  loader_control_interfaces::msg::VehicleState estimate(
    const loader_control_interfaces::msg::VehicleState & predicted,
    const loader_control_interfaces::msg::VehicleState & measured);

private:
  double wheelbase_{2.8};
  double tau_v_{0.8};
  double tau_delta_{0.3};
  
  std::vector<double> process_noise_;
  std::vector<double> measurement_noise_;
  
  // State: [x, y, theta, v, delta]
  std::vector<double> state_mean_;
  std::vector<std::vector<double>> state_covariance_;
};

} // namespace loader_control


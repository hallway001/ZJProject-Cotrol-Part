#pragma once
#include "loader_control_interfaces/msg/vehicle_state.hpp"
#include "nav_msgs/msg/path.hpp"
#include <memory>

namespace loader_control {

class SafetyMonitor {
public:
  SafetyMonitor() = default;
  
  void setMaxLateralError(double max_error) { max_lateral_error_ = max_error; }
  void setMaxHeadingError(double max_error) { max_heading_error_ = max_error; }
  
  bool checkSafety(const loader_control_interfaces::msg::VehicleState & state,
                  const nav_msgs::msg::Path & reference_path);
  
  double getLateralError() const { return last_lateral_error_; }
  double getHeadingError() const { return last_heading_error_; }

private:
  double max_lateral_error_{1.5};
  double max_heading_error_{1.0};
  double last_lateral_error_{0.0};
  double last_heading_error_{0.0};
  
  double computeLateralError(const loader_control_interfaces::msg::VehicleState & state,
                            const nav_msgs::msg::Path & path);
  double computeHeadingError(const loader_control_interfaces::msg::VehicleState & state,
                            const nav_msgs::msg::Path & path);
};

} // namespace loader_control


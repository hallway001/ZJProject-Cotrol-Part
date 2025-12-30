#include "loader_control/utils/safety_monitor.hpp"
#include "loader_control/utils/trajectory_interpolator.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace loader_control {

bool SafetyMonitor::checkSafety(const loader_control_interfaces::msg::VehicleState & state,
                                const nav_msgs::msg::Path & reference_path) {
  last_lateral_error_ = computeLateralError(state, reference_path);
  last_heading_error_ = computeHeadingError(state, reference_path);
  
  if (std::abs(last_lateral_error_) > max_lateral_error_) {
    return false;
  }
  
  if (std::abs(last_heading_error_) > max_heading_error_) {
    return false;
  }
  
  return true;
}

double SafetyMonitor::computeLateralError(const loader_control_interfaces::msg::VehicleState & state,
                                          const nav_msgs::msg::Path & path) {
  if (path.poses.empty()) {
    return 0.0;
  }

  size_t nearest_idx = TrajectoryInterpolator::findNearestIndex(path, state.pose);
  if (nearest_idx >= path.poses.size()) {
    return 0.0;
  }

  const auto & ref_pose = path.poses[nearest_idx].pose;
  
  double dx = ref_pose.position.x - state.pose.position.x;
  double dy = ref_pose.position.y - state.pose.position.y;
  
  tf2::Quaternion q;
  tf2::fromMsg(state.pose.orientation, q);
  double yaw = tf2::getYaw(q);
  
  double lateral_error = -dx * sin(yaw) + dy * cos(yaw);
  
  return lateral_error;
}

double SafetyMonitor::computeHeadingError(const loader_control_interfaces::msg::VehicleState & state,
                                          const nav_msgs::msg::Path & path) {
  if (path.poses.empty()) {
    return 0.0;
  }

  size_t nearest_idx = TrajectoryInterpolator::findNearestIndex(path, state.pose);
  if (nearest_idx >= path.poses.size()) {
    return 0.0;
  }

  const auto & ref_pose = path.poses[nearest_idx].pose;
  
  tf2::Quaternion q_current;
  tf2::fromMsg(state.pose.orientation, q_current);
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


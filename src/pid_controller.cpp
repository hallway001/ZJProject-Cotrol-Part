#include "loader_control/pid_controller.hpp"
#include <cmath>
#include <algorithm>

namespace loader_control
{

PIDController::PIDController(const Config & config)
: config_(config)
{
}

bool PIDController::initialize()
{
  reset();
  return true;
}

ControlCommand PIDController::computeCommand(const VehicleState & state, double dt)
{
  ControlCommand cmd;
  cmd.stamp = state.stamp;
  
  if (dt <= 0.0) {
    return cmd;
  }
  
  // Calculate longitudinal error (velocity tracking)
  double long_error = calculateLongitudinalError(state);
  
  // Calculate lateral error (heading/position tracking)
  double lat_error = calculateLateralError(state);
  
  // Compute longitudinal PID
  if (config_.longitudinal.enable) {
    cmd.velocity_command = computePID(
      long_error, dt,
      config_.longitudinal.kp,
      config_.longitudinal.ki,
      config_.longitudinal.kd,
      long_integral_,
      long_last_error_,
      config_.longitudinal.max_integral,
      config_.longitudinal.anti_windup);
    long_last_output_ = cmd.velocity_command;
  } else {
    // Use desired velocity from path if available
    size_t closest_idx = findClosestPathPoint(state);
    cmd.velocity_command = getDesiredVelocity(state, closest_idx);
  }
  
  // Compute lateral PID
  if (config_.lateral.enable) {
    cmd.steering_command = computePID(
      lat_error, dt,
      config_.lateral.kp,
      config_.lateral.ki,
      config_.lateral.kd,
      lat_integral_,
      lat_last_error_,
      config_.lateral.max_integral,
      config_.lateral.anti_windup);
    
    // Apply steering error compensation (feedforward)
    if (config_.steering_compensation.enable) {
      double compensation = config_.steering_compensation.ff_gain * state.steering_error;
      cmd.steering_command += compensation;
    }
    
    lat_last_output_ = cmd.steering_command;
  }
  
  // Saturate command
  saturateCommand(cmd);
  
  return cmd;
}

void PIDController::reset()
{
  long_integral_ = 0.0;
  long_last_error_ = 0.0;
  long_last_output_ = 0.0;
  lat_integral_ = 0.0;
  lat_last_error_ = 0.0;
  lat_last_output_ = 0.0;
}

void PIDController::setConfig(const Config & config)
{
  config_ = config;
}

double PIDController::computePID(
  double error, double dt,
  double kp, double ki, double kd,
  double & integral, double & last_error,
  double max_integral, bool anti_windup)
{
  // Proportional term
  double p_term = kp * error;
  
  // Integral term
  integral += error * dt;
  if (anti_windup) {
    integral = std::clamp(integral, -max_integral, max_integral);
  }
  double i_term = ki * integral;
  
  // Derivative term
  double derivative = (error - last_error) / (dt + 1e-6);
  double d_term = kd * derivative;
  last_error = error;
  
  // PID output
  double output = p_term + i_term + d_term;
  
  return output;
}

double PIDController::calculateLongitudinalError(const VehicleState & state)
{
  size_t closest_idx = findClosestPathPoint(state);
  double desired_velocity = getDesiredVelocity(state, closest_idx);
  double current_velocity = state.vx;
  return desired_velocity - current_velocity;
}

double PIDController::calculateLateralError(const VehicleState & state)
{
  if (state.ref_path.poses.empty()) {
    return 0.0;
  }
  
  size_t closest_idx = findClosestPathPoint(state);
  if (closest_idx >= state.ref_path.poses.size()) {
    return 0.0;
  }
  
  // Calculate heading error
  double desired_heading = getDesiredHeading(state, closest_idx);
  double heading_error = desired_heading - state.yaw;
  
  // Normalize to [-π, π]
  while (heading_error > M_PI) {
    heading_error -= 2.0 * M_PI;
  }
  while (heading_error < -M_PI) {
    heading_error += 2.0 * M_PI;
  }
  
  // Optionally add lateral position error
  const auto & target_pose = state.ref_path.poses[closest_idx].pose.position;
  double dx = target_pose.x - state.x;
  double dy = target_pose.y - state.y;
  
  // Project onto vehicle's lateral direction
  double lateral_error = -dx * std::sin(state.yaw) + dy * std::cos(state.yaw);
  
  // Combine heading and lateral error (weighted)
  double combined_error = 0.7 * heading_error + 0.3 * lateral_error;
  
  return combined_error;
}

size_t PIDController::findClosestPathPoint(const VehicleState & state)
{
  if (state.ref_path.poses.empty()) {
    return 0;
  }
  
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_idx = 0;
  
  for (size_t i = 0; i < state.ref_path.poses.size(); ++i) {
    const auto & pose = state.ref_path.poses[i].pose.position;
    double dx = pose.x - state.x;
    double dy = pose.y - state.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  
  return closest_idx;
}

double PIDController::getDesiredVelocity(const VehicleState & state, size_t point_idx)
{
  if (state.ref_path.poses.empty() || point_idx >= state.ref_path.poses.size()) {
    return state.vx;  // Maintain current velocity
  }
  
  // Extract velocity from path if available (assuming twist is stored somewhere)
  // For now, return a constant desired velocity or scale based on curvature
  double base_velocity = max_velocity_;
  
  // Reduce velocity based on curvature
  if (state.path_curvature > 0.1) {
    base_velocity = max_velocity_ * (1.0 - state.path_curvature / 0.5);
  }
  
  return std::max(0.5, base_velocity);  // Minimum velocity
}

double PIDController::getDesiredHeading(const VehicleState & state, size_t point_idx)
{
  if (state.ref_path.poses.empty() || point_idx >= state.ref_path.poses.size()) {
    return state.yaw;
  }
  
  // Get heading from quaternion
  const auto & orientation = state.ref_path.poses[point_idx].pose.orientation;
  double qx = orientation.x;
  double qy = orientation.y;
  double qz = orientation.z;
  double qw = orientation.w;
  
  // Convert quaternion to yaw
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  
  // If next point exists, calculate heading from position difference
  if (point_idx < state.ref_path.poses.size() - 1) {
    const auto & p1 = state.ref_path.poses[point_idx].pose.position;
    const auto & p2 = state.ref_path.poses[point_idx + 1].pose.position;
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    if (std::sqrt(dx * dx + dy * dy) > 1e-3) {
      yaw = std::atan2(dy, dx);
    }
  }
  
  return yaw;
}

}  // namespace loader_control


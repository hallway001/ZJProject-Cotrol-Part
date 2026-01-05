#include "loader_control/pure_pursuit_controller.hpp"
#include <cmath>
#include <algorithm>

namespace loader_control
{

PurePursuitController::PurePursuitController(const Config & config)
: config_(config), current_lookahead_(config.lookahead.base_distance)
{
}

bool PurePursuitController::initialize()
{
  current_lookahead_ = config_.lookahead.base_distance;
  return true;
}

ControlCommand PurePursuitController::computeCommand(const VehicleState & state, double dt)
{
  (void)dt;  // Suppress unused parameter warning
  ControlCommand cmd;
  cmd.stamp = state.stamp;
  
  if (state.ref_path.poses.empty()) {
    return cmd;
  }
  
  // Calculate dynamic lookahead distance
  double lookahead_dist = calculateLookaheadDistance(state);
  current_lookahead_ = lookahead_dist;
  
  // Find lookahead point on path
  size_t lookahead_idx;
  double lookahead_x, lookahead_y;
  bool found = findLookaheadPoint(state, lookahead_dist, lookahead_idx, lookahead_x, lookahead_y);
  
  if (!found) {
    // If lookahead point not found, use closest point
    cmd.steering_command = 0.0;
    cmd.velocity_command = state.vx;
    return cmd;
  }
  
  // Compute steering angle using pure pursuit
  cmd.steering_command = computeSteeringAngle(state, lookahead_x, lookahead_y);
  
  // Get desired velocity from path
  cmd.velocity_command = getDesiredVelocity(state, lookahead_idx);
  
  // Saturate command
  saturateCommand(cmd);
  
  return cmd;
}

void PurePursuitController::reset()
{
  current_lookahead_ = config_.lookahead.base_distance;
}

void PurePursuitController::setConfig(const Config & config)
{
  config_ = config;
  current_lookahead_ = config_.lookahead.base_distance;
}

double PurePursuitController::calculateLookaheadDistance(const VehicleState & state)
{
  // Base lookahead distance
  double lookahead = config_.lookahead.base_distance;
  
  // Adjust based on velocity
  double velocity_adjustment = config_.lookahead.velocity_gain * std::abs(state.vx);
  lookahead += velocity_adjustment;
  
  // Adjust based on steering error (error compensation)
  if (config_.error_compensation.enable) {
    double error_compensation = config_.error_compensation.lookahead_error_gain *
                                std::abs(state.steering_error);
    error_compensation = std::min(error_compensation, config_.error_compensation.max_error_compensation);
    lookahead += error_compensation;
  }
  
  // Clamp to limits
  lookahead = std::clamp(lookahead,
    config_.lookahead.min_distance,
    config_.lookahead.max_distance);
  
  return lookahead;
}

bool PurePursuitController::findLookaheadPoint(
  const VehicleState & state,
  double lookahead_dist,
  size_t & lookahead_idx,
  double & lookahead_x,
  double & lookahead_y)
{
  if (state.ref_path.poses.empty()) {
    return false;
  }
  
  // Start searching from closest point
  size_t start_idx = std::max(0, static_cast<int>(state.closest_path_index));
  
  // Find the first point on path that is at least lookahead_dist away
  for (size_t i = start_idx; i < state.ref_path.poses.size(); ++i) {
    const auto & pose = state.ref_path.poses[i].pose.position;
    double dx = pose.x - state.x;
    double dy = pose.y - state.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    if (dist >= lookahead_dist) {
      lookahead_idx = i;
      lookahead_x = pose.x;
      lookahead_y = pose.y;
      return true;
    }
  }
  
  // If no point found, use the last point
  if (!state.ref_path.poses.empty()) {
    const auto & last_pose = state.ref_path.poses.back().pose.position;
    lookahead_idx = state.ref_path.poses.size() - 1;
    lookahead_x = last_pose.x;
    lookahead_y = last_pose.y;
    return true;
  }
  
  return false;
}

double PurePursuitController::computeSteeringAngle(
  const VehicleState & state,
  double lookahead_x,
  double lookahead_y)
{
  // Vector from vehicle to lookahead point (in vehicle frame)
  double dx = lookahead_x - state.x;
  double dy = lookahead_y - state.y;
  
  // Transform to vehicle frame
  double cos_yaw = std::cos(state.yaw);
  double sin_yaw = std::sin(state.yaw);
  double local_x = dx * cos_yaw + dy * sin_yaw;
  double local_y = -dx * sin_yaw + dy * cos_yaw;
  
  // Distance to lookahead point
  double dist = std::sqrt(local_x * local_x + local_y * local_y);
  
  if (dist < 1e-3) {
    return 0.0;
  }
  
  // Pure pursuit geometry: α = atan2(2 * L * sin(α), ld)
  // where L is wheelbase (assume 2.5m for loader), ld is lookahead distance
  // For bicycle model: δ = atan(2 * L * sin(α) / ld)
  // where α is angle to lookahead point
  
  double wheelbase = 2.5;  // TODO: Make configurable
  double alpha = std::atan2(local_y, local_x);
  
  // Pure pursuit steering angle
  double steering = std::atan2(2.0 * wheelbase * std::sin(alpha), dist);
  
  // Check curvature constraint
  double curvature = std::abs(steering) / wheelbase;
  if (curvature > config_.curvature.max_curvature) {
    steering = std::copysign(config_.curvature.max_curvature * wheelbase, steering);
  }
  
  return steering;
}

double PurePursuitController::getDesiredVelocity(const VehicleState & state, size_t point_idx)
{
  if (state.ref_path.poses.empty() || point_idx >= state.ref_path.poses.size()) {
    return state.vx;
  }
  
  // Base velocity
  double base_velocity = max_velocity_;
  
  // Reduce velocity based on curvature
  double curvature = calculateCurvature(state.ref_path, point_idx);
  if (curvature > 0.1) {
    base_velocity = max_velocity_ * (1.0 - curvature / config_.curvature.max_curvature);
  }
  
  // Reduce velocity based on lookahead distance (shorter lookahead = slower)
  if (current_lookahead_ < config_.lookahead.base_distance) {
    base_velocity *= (current_lookahead_ / config_.lookahead.base_distance);
  }
  
  return std::max(0.5, base_velocity);  // Minimum velocity
}

double PurePursuitController::distance(double x1, double y1, double x2, double y2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

double PurePursuitController::calculateCurvature(
  const nav_msgs::msg::Path & path,
  size_t idx)
{
  if (path.poses.size() < 3 || idx == 0 || idx >= path.poses.size() - 1) {
    return 0.0;
  }
  
  // Use three points to calculate curvature
  const auto & p0 = path.poses[idx - 1].pose.position;
  const auto & p1 = path.poses[idx].pose.position;
  const auto & p2 = path.poses[idx + 1].pose.position;
  
  double dx1 = p1.x - p0.x;
  double dy1 = p1.y - p0.y;
  double dx2 = p2.x - p1.x;
  double dy2 = p2.y - p1.y;
  
  double ds1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
  double ds2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
  
  if (ds1 < 1e-6 || ds2 < 1e-6) {
    return 0.0;
  }
  
  // Normalize
  dx1 /= ds1;
  dy1 /= ds1;
  dx2 /= ds2;
  dy2 /= ds2;
  
  // Curvature = cross product / average distance
  double cross = dx1 * dy2 - dy1 * dx2;
  double curvature = std::abs(cross) / ((ds1 + ds2) / 2.0);
  
  return curvature;
}

}  // namespace loader_control


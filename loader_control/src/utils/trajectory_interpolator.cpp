#include "loader_control/utils/trajectory_interpolator.hpp"
#include <cmath>
#include <limits>

namespace loader_control {

size_t TrajectoryInterpolator::findNearestIndex(const nav_msgs::msg::Path & path,
                                                const geometry_msgs::msg::Pose & pose) {
  if (path.poses.empty()) {
    return 0;
  }

  double min_dist = std::numeric_limits<double>::max();
  size_t nearest_idx = 0;

  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - pose.position.x;
    double dy = path.poses[i].pose.position.y - pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  return nearest_idx;
}

geometry_msgs::msg::Pose TrajectoryInterpolator::interpolate(const nav_msgs::msg::Path & path,
                                                             size_t idx,
                                                             double fraction) {
  geometry_msgs::msg::Pose result;
  
  if (path.poses.empty()) {
    return result;
  }
  
  if (idx >= path.poses.size() - 1) {
    return path.poses.back().pose;
  }
  
  // 线性插值
  const auto & p1 = path.poses[idx].pose;
  const auto & p2 = path.poses[idx + 1].pose;
  
  result.position.x = p1.position.x + fraction * (p2.position.x - p1.position.x);
  result.position.y = p1.position.y + fraction * (p2.position.y - p1.position.y);
  result.position.z = p1.position.z + fraction * (p2.position.z - p1.position.z);
  
  // 四元数插值（简化版，实际应使用slerp）
  result.orientation = p1.orientation;
  
  return result;
}

} // namespace loader_control


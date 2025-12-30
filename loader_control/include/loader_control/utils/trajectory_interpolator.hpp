#pragma once
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace loader_control {

class TrajectoryInterpolator {
public:
  static size_t findNearestIndex(const nav_msgs::msg::Path & path, 
                                 const geometry_msgs::msg::Pose & pose);
  
  static geometry_msgs::msg::Pose interpolate(const nav_msgs::msg::Path & path,
                                              size_t idx, 
                                              double fraction);
};

} // namespace loader_control


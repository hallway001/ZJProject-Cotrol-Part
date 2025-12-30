#include "loader_control/controllers/pure_pursuit_controller.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace loader_control {

void PurePursuitController::configure(const rclcpp::Node::SharedPtr & node) {
  node->declare_parameter("controller.pure_pursuit.k_lookahead", 1.5);
  node->declare_parameter("controller.pure_pursuit.min_lookahead", 2.0);
  node->declare_parameter("controller.pure_pursuit.ff_weight", 0.3);
  node->declare_parameter("max_steering", 0.6);
  node->declare_parameter("max_throttle", 0.8);
  node->declare_parameter("wheelbase", 2.8);

  k_lookahead_ = node->get_parameter("controller.pure_pursuit.k_lookahead").as_double();
  min_lookahead_ = node->get_parameter("controller.pure_pursuit.min_lookahead").as_double();
  ff_weight_ = node->get_parameter("controller.pure_pursuit.ff_weight").as_double();
  max_steering_ = node->get_parameter("max_steering").as_double();
  max_throttle_ = node->get_parameter("max_throttle").as_double();
  wheelbase_ = node->get_parameter("wheelbase").as_double();
}

ControllerOutput PurePursuitController::compute(const ControllerInput & input) {
  ControllerOutput output;

  if (input.reference_path.poses.empty()) {
    output.throttle_cmd = 0.0;
    output.steering_cmd = 0.0;
    return output;
  }

  // 计算前瞻距离
  double lookahead_dist = std::max(min_lookahead_, k_lookahead_ * input.current_speed);

  // 查找前瞻点
  size_t lookahead_idx = findLookaheadPoint(input.reference_path, 
                                           input.current_pose.pose, 
                                           lookahead_dist);

  if (lookahead_idx >= input.reference_path.poses.size()) {
    lookahead_idx = input.reference_path.poses.size() - 1;
  }

  const auto & target_pose = input.reference_path.poses[lookahead_idx].pose;

  // 计算转向角
  double steering_angle = computeSteeringAngle(input.current_pose.pose, 
                                              target_pose, 
                                              lookahead_dist);

  // 前馈项（根据路径曲率）
  double curvature = 0.0;
  if (lookahead_idx > 0 && lookahead_idx < input.reference_path.poses.size()) {
    // 简化曲率计算
    const auto & prev = input.reference_path.poses[lookahead_idx - 1].pose.position;
    const auto & curr = input.reference_path.poses[lookahead_idx].pose.position;
    double dx = curr.x - prev.x;
    double dy = curr.y - prev.y;
    // 这里可以添加更复杂的曲率计算
  }

  double ff_steering = ff_weight_ * std::atan(wheelbase_ * curvature);
  output.steering_cmd = steering_angle + ff_steering;
  output.steering_cmd = std::max(-max_steering_, std::min(max_steering_, output.steering_cmd));

  // 速度控制（简单实现）
  output.throttle_cmd = max_throttle_ * 0.7;  // 可以根据需要调整

  return output;
}

void PurePursuitController::reset() {
  // Pure Pursuit无状态，无需重置
}

size_t PurePursuitController::findLookaheadPoint(const nav_msgs::msg::Path & path,
                                                 const geometry_msgs::msg::Pose & pose,
                                                 double lookahead_dist) {
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_idx = 0;

  // 找到最近点
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - pose.position.x;
    double dy = path.poses[i].pose.position.y - pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // 从最近点开始，找到距离为lookahead_dist的点
  for (size_t i = closest_idx; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - pose.position.x;
    double dy = path.poses[i].pose.position.y - pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    if (dist >= lookahead_dist) {
      return i;
    }
  }

  return path.poses.size() - 1;
}

double PurePursuitController::computeSteeringAngle(const geometry_msgs::msg::Pose & current_pose,
                                                   const geometry_msgs::msg::Pose & target_pose,
                                                   double lookahead_dist) {
  // 计算目标点相对当前位置的向量
  double dx = target_pose.position.x - current_pose.position.x;
  double dy = target_pose.position.y - current_pose.position.y;
  
  // 获取当前朝向
  tf2::Quaternion q;
  tf2::fromMsg(current_pose.orientation, q);
  double yaw = tf2::getYaw(q);
  
  // 计算目标点在车辆坐标系中的位置
  double local_x = dx * cos(yaw) + dy * sin(yaw);
  double local_y = -dx * sin(yaw) + dy * cos(yaw);
  
  // Pure Pursuit公式
  double alpha = std::atan2(local_y, local_x);
  double steering_angle = std::atan2(2.0 * wheelbase_ * sin(alpha), lookahead_dist);
  
  return steering_angle;
}

} // namespace loader_control


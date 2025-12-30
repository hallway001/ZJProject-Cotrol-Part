#pragma once
#include "loader_control/controllers/base_controller.hpp"
#include <memory>

namespace loader_control {

class PurePursuitController : public BaseController {
public:
  PurePursuitController() = default;
  ~PurePursuitController() override = default;

  void configure(const rclcpp::Node::SharedPtr & node) override;
  ControllerOutput compute(const ControllerInput & input) override;
  void reset() override;

private:
  double k_lookahead_{1.5};
  double min_lookahead_{2.0};
  double ff_weight_{0.3};
  double max_steering_{0.6};
  double max_throttle_{0.8};
  double wheelbase_{2.8};

  size_t findLookaheadPoint(const nav_msgs::msg::Path & path, 
                            const geometry_msgs::msg::Pose & pose, 
                            double lookahead_dist);
  double computeSteeringAngle(const geometry_msgs::msg::Pose & current_pose,
                             const geometry_msgs::msg::Pose & target_pose,
                             double lookahead_dist);
};

} // namespace loader_control


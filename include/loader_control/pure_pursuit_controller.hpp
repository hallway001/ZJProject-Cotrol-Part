#ifndef LOADER_CONTROL__PURE_PURSUIT_CONTROLLER_HPP_
#define LOADER_CONTROL__PURE_PURSUIT_CONTROLLER_HPP_

#include "loader_control/controller_base.hpp"
#include <nav_msgs/msg/path.hpp>
#include <string>

namespace loader_control
{

/**
 * @brief Pure Pursuit Controller for path tracking
 * Uses lookahead distance to compute steering angle
 * Supports dynamic lookahead adjustment based on steering error
 */
class PurePursuitController : public ControllerBase
{
public:
  struct Config
  {
    double update_rate = 50.0;
    
    // Lookahead distance
    struct {
      double base_distance = 2.0;      // Base lookahead distance (m)
      double min_distance = 1.0;       // Minimum lookahead (m)
      double max_distance = 5.0;       // Maximum lookahead (m)
      double velocity_gain = 0.5;      // Lookahead velocity gain
    } lookahead;
    
    // Error compensation
    struct {
      bool enable = true;
      double lookahead_error_gain = 0.5;  // Lookahead adjustment based on error
      double max_error_compensation = 1.0; // Maximum additional lookahead from error (m)
    } error_compensation;
    
    // Curvature limits
    struct {
      double min_radius = 2.0;          // Minimum turning radius (m)
      double max_curvature = 0.5;       // Maximum curvature (1/m)
    } curvature;
  };
  
  explicit PurePursuitController(const Config & config);
  
  bool initialize() override;
  ControlCommand computeCommand(const VehicleState & state, double dt) override;
  void reset() override;
  std::string getName() const override { return "PurePursuit"; }
  
  void setConfig(const Config & config);

private:
  Config config_;
  
  // Current lookahead distance (dynamically adjusted)
  double current_lookahead_ = 2.0;
  
  /**
   * @brief Calculate dynamic lookahead distance
   */
  double calculateLookaheadDistance(const VehicleState & state);
  
  /**
   * @brief Find lookahead point on path
   */
  bool findLookaheadPoint(
    const VehicleState & state,
    double lookahead_dist,
    size_t & lookahead_idx,
    double & lookahead_x,
    double & lookahead_y);
  
  /**
   * @brief Compute steering angle using pure pursuit
   */
  double computeSteeringAngle(
    const VehicleState & state,
    double lookahead_x,
    double lookahead_y);
  
  /**
   * @brief Get desired velocity from path
   */
  double getDesiredVelocity(const VehicleState & state, size_t point_idx);
  
  /**
   * @brief Calculate distance between two points
   */
  double distance(double x1, double y1, double x2, double y2);
  
  /**
   * @brief Calculate path curvature at a point
   */
  double calculateCurvature(
    const nav_msgs::msg::Path & path,
    size_t idx);
};

}  // namespace loader_control

#endif  // LOADER_CONTROL__PURE_PURSUIT_CONTROLLER_HPP_


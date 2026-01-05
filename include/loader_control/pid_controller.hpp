#ifndef LOADER_CONTROL__PID_CONTROLLER_HPP_
#define LOADER_CONTROL__PID_CONTROLLER_HPP_

#include "loader_control/controller_base.hpp"
#include <string>

namespace loader_control
{

/**
 * @brief PID Controller for path tracking
 * Provides longitudinal (velocity) and lateral (steering) PID control
 * with steering error compensation
 */
class PIDController : public ControllerBase
{
public:
  struct Config
  {
    double update_rate = 50.0;
    
    // Longitudinal PID
    struct {
      bool enable = true;
      double kp = 1.5;
      double ki = 0.1;
      double kd = 0.3;
      double max_integral = 1.0;
      bool anti_windup = true;
    } longitudinal;
    
    // Lateral PID
    struct {
      bool enable = true;
      double kp = 2.0;
      double ki = 0.05;
      double kd = 0.5;
      double max_integral = 0.5;
      bool anti_windup = true;
    } lateral;
    
    // Steering error compensation
    struct {
      bool enable = true;
      double ff_gain = 0.3;
    } steering_compensation;
  };
  
  explicit PIDController(const Config & config);
  
  bool initialize() override;
  ControlCommand computeCommand(const VehicleState & state, double dt) override;
  void reset() override;
  std::string getName() const override { return "PID"; }
  
  void setConfig(const Config & config);

private:
  Config config_;
  
  // Longitudinal PID state
  double long_integral_ = 0.0;
  double long_last_error_ = 0.0;
  double long_last_output_ = 0.0;
  
  // Lateral PID state
  double lat_integral_ = 0.0;
  double lat_last_error_ = 0.0;
  double lat_last_output_ = 0.0;
  
  /**
   * @brief Compute PID output
   */
  double computePID(
    double error, double dt,
    double kp, double ki, double kd,
    double & integral, double & last_error,
    double max_integral, bool anti_windup);
  
  /**
   * @brief Calculate longitudinal velocity error
   */
  double calculateLongitudinalError(const VehicleState & state);
  
  /**
   * @brief Calculate lateral/heading error
   */
  double calculateLateralError(const VehicleState & state);
  
  /**
   * @brief Find closest point on path
   */
  size_t findClosestPathPoint(const VehicleState & state);
  
  /**
   * @brief Get desired velocity from path
   */
  double getDesiredVelocity(const VehicleState & state, size_t point_idx);
  
  /**
   * @brief Get desired heading from path
   */
  double getDesiredHeading(const VehicleState & state, size_t point_idx);
};

}  // namespace loader_control

#endif  // LOADER_CONTROL__PID_CONTROLLER_HPP_


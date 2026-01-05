#ifndef LOADER_CONTROL__CONTROLLER_BASE_HPP_
#define LOADER_CONTROL__CONTROLLER_BASE_HPP_

#include <memory>
#include <string>
#include "loader_control/vehicle_state.hpp"
#include "loader_control/actuator_model.hpp"

namespace loader_control
{

/**
 * @brief Control command structure
 */
struct ControlCommand
{
  double velocity_command = 0.0;    // Longitudinal velocity command (m/s)
  double steering_command = 0.0;    // Steering angle command (rad)
  
  rclcpp::Time stamp;
  
  void reset()
  {
    velocity_command = 0.0;
    steering_command = 0.0;
  }
};

/**
 * @brief Base class for all path tracking controllers
 * Provides unified interface for PID, Pure Pursuit, MPC, etc.
 */
class ControllerBase
{
public:
  ControllerBase() = default;
  virtual ~ControllerBase() = default;
  
  /**
   * @brief Initialize controller with parameters
   */
  virtual bool initialize() = 0;
  
  /**
   * @brief Compute control command based on current state
   * @param state Current vehicle state
   * @param dt Time step (s)
   * @return Control command
   */
  virtual ControlCommand computeCommand(const VehicleState & state, double dt) = 0;
  
  /**
   * @brief Reset controller internal state
   */
  virtual void reset() = 0;
  
  /**
   * @brief Get controller name
   */
  virtual std::string getName() const = 0;
  
  /**
   * @brief Set actuator models for delay/error simulation
   */
  void setActuatorModels(
    std::shared_ptr<ActuatorModel> steering_model,
    std::shared_ptr<ActuatorModel> throttle_model)
  {
    steering_actuator_ = steering_model;
    throttle_actuator_ = throttle_model;
  }
  
  /**
   * @brief Apply actuator models to control command
   * This applies delay, dead zone, errors, etc. to simulate realistic behavior
   */
  ControlCommand applyActuatorModels(const ControlCommand & command, double dt);
  
protected:
  // Actuator models (optional, for simulation/testing)
  std::shared_ptr<ActuatorModel> steering_actuator_;
  std::shared_ptr<ActuatorModel> throttle_actuator_;
  
  // Output limits
  double max_velocity_ = 3.0;
  double max_steering_ = 0.785;
  
  /**
   * @brief Saturate control command
   */
  void saturateCommand(ControlCommand & cmd);
};

}  // namespace loader_control

#endif  // LOADER_CONTROL__CONTROLLER_BASE_HPP_


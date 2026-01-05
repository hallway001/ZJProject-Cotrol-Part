#include "loader_control/controller_base.hpp"
#include <algorithm>

namespace loader_control
{

ControlCommand ControllerBase::applyActuatorModels(
  const ControlCommand & command, double dt)
{
  ControlCommand output = command;
  
  // Apply throttle actuator model if available
  if (throttle_actuator_) {
    output.velocity_command = throttle_actuator_->update(command.velocity_command, dt);
  }
  
  // Apply steering actuator model if available
  if (steering_actuator_) {
    output.steering_command = steering_actuator_->update(command.steering_command, dt);
  }
  
  // Saturate command
  saturateCommand(output);
  
  return output;
}

void ControllerBase::saturateCommand(ControlCommand & cmd)
{
  // Saturate velocity command
  cmd.velocity_command = std::clamp(cmd.velocity_command, 0.0, max_velocity_);
  
  // Saturate steering command
  cmd.steering_command = std::clamp(cmd.steering_command, -max_steering_, max_steering_);
}

}  // namespace loader_control


#include "loader_control/actuator_model.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

namespace loader_control
{

ActuatorModel::ActuatorModel(const Config & config, const std::string & name)
: name_(name), config_(config),
  current_command_(0.0), current_output_(0.0), last_command_(0.0),
  rng_(std::chrono::steady_clock::now().time_since_epoch().count()),
  noise_dist_(0.0, config.noise_std)
{
}

double ActuatorModel::update(double command, double dt)
{
  if (dt <= 0.0) {
    return current_output_;
  }
  
  // Apply first-order delay
  double delayed_cmd = applyDelay(command, dt);
  
  // Apply saturation limits
  double saturated_cmd = applySaturation(delayed_cmd);
  
  // Apply low-pass filter
  double filtered_cmd = applyLowPassFilter(saturated_cmd, dt);
  
  // Apply dead zone
  double cmd_after_deadzone = applyDeadZone(filtered_cmd);
  
  // Apply scale bias
  double cmd_after_scale = applyScaleBias(cmd_after_deadzone);
  
  // Add noise
  current_output_ = addNoise(cmd_after_scale);
  
  // Update internal state
  current_command_ = filtered_cmd;
  last_command_ = command;
  
  return current_output_;
}

void ActuatorModel::reset()
{
  current_command_ = 0.0;
  current_output_ = 0.0;
  last_command_ = 0.0;
}

void ActuatorModel::setConfig(const Config & config)
{
  config_ = config;
  noise_dist_ = std::normal_distribution<double>(0.0, config.noise_std);
}

double ActuatorModel::applyDelay(double command, double dt)
{
  // First-order delay: τ * dδ/dt + δ = δ_cmd
  // Discrete: δ[k+1] = δ[k] + (dt/τ) * (δ_cmd - δ[k])
  if (config_.delay_time_constant > 1e-6) {
    double alpha = dt / (config_.delay_time_constant + dt);
    current_command_ = alpha * command + (1.0 - alpha) * current_command_;
    return current_command_;
  }
  return command;
}

double ActuatorModel::applyDeadZone(double value)
{
  if (std::abs(value) < config_.dead_zone) {
    return 0.0;
  }
  // Linear mapping: output = (input - dead_zone) / (1 - dead_zone) when input > dead_zone
  if (value > config_.dead_zone) {
    return (value - config_.dead_zone) / (1.0 - config_.dead_zone);
  } else {
    return (value + config_.dead_zone) / (1.0 - config_.dead_zone);
  }
}

double ActuatorModel::applyScaleBias(double value)
{
  return config_.scale_bias * value;
}

double ActuatorModel::applySaturation(double value)
{
  return std::clamp(value, config_.min_value, config_.max_value);
}

double ActuatorModel::applyLowPassFilter(double value, double dt)
{
  // First-order low-pass filter: y[k] = α * x[k] + (1-α) * y[k-1]
  // α = dt / (dt + τ_filter), where τ_filter is derived from filter_alpha
  // For simplicity, use filter_alpha directly
  double alpha = config_.filter_alpha;
  current_command_ = alpha * value + (1.0 - alpha) * current_command_;
  return current_command_;
}

double ActuatorModel::addNoise(double value)
{
  if (config_.noise_std > 1e-6) {
    return value + noise_dist_(rng_);
  }
  return value;
}

}  // namespace loader_control


#ifndef LOADER_CONTROL__ACTUATOR_MODEL_HPP_
#define LOADER_CONTROL__ACTUATOR_MODEL_HPP_

#include <string>
#include <random>

namespace loader_control
{

/**
 * @brief Actuator physical model with delay, dead zone, and error modeling
 * Simulates realistic actuator behavior including:
 * - First-order delay response
 * - Dead zone
 * - Scale bias
 * - Random noise
 */
class ActuatorModel
{
public:
  struct Config
  {
    double delay_time_constant = 0.15;  // First-order delay τ (s)
    double dead_zone = 0.02;            // Dead zone width (±)
    double scale_bias = 1.0;            // Scale bias
    double noise_std = 0.0;             // Noise standard deviation
    double max_value = 1.0;             // Maximum output value
    double min_value = -1.0;            // Minimum output value
    double filter_alpha = 0.8;          // Low-pass filter coefficient
  };
  
  explicit ActuatorModel(const Config & config, const std::string & name = "actuator");
  
  /**
   * @brief Update actuator model with command
   * @param command Raw command input
   * @param dt Time step (s)
   * @return Actual output value (with delay, dead zone, error, etc.)
   */
  double update(double command, double dt);
  
  /**
   * @brief Get current output value
   */
  double getOutput() const { return current_output_; }
  
  /**
   * @brief Get current command (after delay filter)
   */
  double getCommand() const { return current_command_; }
  
  /**
   * @brief Reset actuator state
   */
  void reset();
  
  /**
   * @brief Update configuration
   */
  void setConfig(const Config & config);
  
  /**
   * @brief Get configuration
   */
  const Config & getConfig() const { return config_; }

private:
  std::string name_;
  Config config_;
  
  // Internal state
  double current_command_;    // Command after delay filter
  double current_output_;     // Final output with all effects
  double last_command_;       // Previous command for filter
  
  // Random number generator for noise
  std::mt19937 rng_;
  std::normal_distribution<double> noise_dist_;
  
  /**
   * @brief Apply first-order delay
   */
  double applyDelay(double command, double dt);
  
  /**
   * @brief Apply dead zone
   */
  double applyDeadZone(double value);
  
  /**
   * @brief Apply scale bias
   */
  double applyScaleBias(double value);
  
  /**
   * @brief Apply saturation limits
   */
  double applySaturation(double value);
  
  /**
   * @brief Apply low-pass filter
   */
  double applyLowPassFilter(double value, double dt);
  
  /**
   * @brief Add noise
   */
  double addNoise(double value);
};

}  // namespace loader_control

#endif  // LOADER_CONTROL__ACTUATOR_MODEL_HPP_


#ifndef LOADER_CONTROL__STEERING_ERROR_MONITOR_HPP_
#define LOADER_CONTROL__STEERING_ERROR_MONITOR_HPP_

#include <memory>
#include <deque>
#include <mutex>
#include <string>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "loader_control/vehicle_state.hpp"

namespace loader_control
{

/**
 * @brief Steering error monitoring and statistics
 * Calculates, logs, and visualizes steering error metrics
 */
class SteeringErrorMonitor
{
public:
  struct Config
  {
    double update_rate = 50.0;
    
    // Statistics window
    struct {
      double window_duration = 2.0;    // Statistics window (s)
      bool enable_rmse = true;
      bool enable_max_error = true;
      bool enable_integral = true;
    } statistics;
    
    // Thresholds
    struct {
      double warning_rmse = 0.03;      // rad (≈1.7°)
      double error_rmse = 0.05;        // rad (≈2.9°)
      double critical_rmse = 0.1;      // rad (≈5.7°)
      double warning_duration = 1.0;   // s
    } thresholds;
    
    // Logging
    struct {
      bool enable = true;
      std::string log_dir = "logs";
      std::string log_filename = "steering_error_%Y%m%d_%H%M%S.csv";
    } logging;
    
    // Visualization
    struct {
      bool enable = true;
      std::string topic = "/loader/control/steering_error_vis";
    } visualization;
  };
  
  struct ErrorStatistics
  {
    double rmse = 0.0;              // Root mean square error
    double max_error = 0.0;         // Maximum absolute error
    double mean_error = 0.0;        // Mean error
    double integral_error = 0.0;    // Integral of absolute error
    size_t sample_count = 0;
    rclcpp::Time last_update;
  };
  
  SteeringErrorMonitor(
    rclcpp::Node::SharedPtr node,
    const Config & config);
  
  ~SteeringErrorMonitor();
  
  /**
   * @brief Update monitor with new state
   */
  void update(const VehicleState & state);
  
  /**
   * @brief Get current statistics
   */
  ErrorStatistics getStatistics() const;
  
  /**
   * @brief Get current error
   */
  double getCurrentError() const;
  
  /**
   * @brief Reset statistics
   */
  void reset();
  
  /**
   * @brief Set configuration
   */
  void setConfig(const Config & config);

private:
  rclcpp::Node::SharedPtr node_;
  Config config_;
  
  // Error history (sliding window)
  mutable std::mutex data_mutex_;
  struct ErrorData
  {
    rclcpp::Time stamp;
    double delta_cmd;
    double delta_fb;
    double error;
    double velocity_x;
    double curvature;
  };
  std::deque<ErrorData> error_history_;
  
  // Current statistics
  ErrorStatistics current_stats_;
  double current_error_ = 0.0;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_;
  
  // Logging
  std::ofstream log_file_;
  bool log_enabled_ = false;
  
  /**
   * @brief Calculate statistics from history
   */
  ErrorStatistics calculateStatistics();
  
  /**
   * @brief Update sliding window
   */
  void updateWindow(const ErrorData & data);
  
  /**
   * @brief Publish error
   */
  void publishError(double error);
  
  /**
   * @brief Publish diagnostics
   */
  void publishDiagnostics(const ErrorStatistics & stats);
  
  /**
   * @brief Publish visualization marker
   */
  void publishVisualization(double error);
  
  /**
   * @brief Log to file
   */
  void logToFile(const ErrorData & data, const ErrorStatistics & stats);
  
  /**
   * @brief Initialize log file
   */
  void initializeLogFile();
  
  /**
   * @brief Get color based on error magnitude
   */
  std::array<double, 3> getErrorColor(double error) const;
};

}  // namespace loader_control

#endif  // LOADER_CONTROL__STEERING_ERROR_MONITOR_HPP_


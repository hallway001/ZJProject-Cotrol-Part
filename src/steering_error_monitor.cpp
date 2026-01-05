#include "loader_control/steering_error_monitor.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <filesystem>

namespace loader_control
{

SteeringErrorMonitor::SteeringErrorMonitor(
  rclcpp::Node::SharedPtr node,
  const Config & config)
: node_(node), config_(config)
{
  // Initialize statistics
  current_stats_.rmse = 0.0;
  current_stats_.max_error = 0.0;
  current_stats_.mean_error = 0.0;
  current_stats_.integral_error = 0.0;
  current_stats_.sample_count = 0;
  current_stats_.last_update = node_->now();
  
  // Create publishers
  error_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "/loader/control/steering_error", 10);
  
  diagnostics_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/diagnostics", 10);
  
  if (config_.visualization.enable) {
    vis_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      config_.visualization.topic, 10);
  }
  
  // Initialize logging
  if (config_.logging.enable) {
    initializeLogFile();
  }
}

SteeringErrorMonitor::~SteeringErrorMonitor()
{
  if (log_file_.is_open()) {
    log_file_.close();
  }
}

void SteeringErrorMonitor::update(const VehicleState & state)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Create error data point
  ErrorData data;
  data.stamp = state.stamp;
  data.delta_cmd = state.delta_cmd;
  data.delta_fb = state.delta_fb;
  data.error = state.steering_error;
  data.velocity_x = state.vx;
  data.curvature = state.path_curvature;
  
  // Update sliding window
  updateWindow(data);
  
  // Calculate statistics
  current_stats_ = calculateStatistics();
  current_error_ = data.error;
  current_stats_.last_update = state.stamp;
  
  // Publish error
  publishError(data.error);
  
  // Publish diagnostics
  publishDiagnostics(current_stats_);
  
  // Publish visualization
  if (config_.visualization.enable) {
    publishVisualization(data.error);
  }
  
  // Log to file
  if (config_.logging.enable && log_file_.is_open()) {
    logToFile(data, current_stats_);
  }
}

SteeringErrorMonitor::ErrorStatistics SteeringErrorMonitor::getStatistics() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return current_stats_;
}

double SteeringErrorMonitor::getCurrentError() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return current_error_;
}

void SteeringErrorMonitor::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  error_history_.clear();
  current_stats_.rmse = 0.0;
  current_stats_.max_error = 0.0;
  current_stats_.mean_error = 0.0;
  current_stats_.integral_error = 0.0;
  current_stats_.sample_count = 0;
  current_error_ = 0.0;
}

void SteeringErrorMonitor::setConfig(const Config & config)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  config_ = config;
  
  // Reinitialize logging if needed
  if (config_.logging.enable && !log_file_.is_open()) {
    initializeLogFile();
  }
}

SteeringErrorMonitor::ErrorStatistics SteeringErrorMonitor::calculateStatistics()
{
  ErrorStatistics stats;
  
  if (error_history_.empty()) {
    return stats;
  }
  
  // Calculate time window
  rclcpp::Time now = node_->now();
  rclcpp::Time window_start = now - rclcpp::Duration::from_seconds(config_.statistics.window_duration);
  
  // Filter data within time window
  std::vector<double> errors;
  double sum_error = 0.0;
  double sum_squared_error = 0.0;
  double max_abs_error = 0.0;
  double integral_error = 0.0;
  size_t count = 0;
  rclcpp::Time last_time = window_start;
  
  for (const auto & data : error_history_) {
    if (data.stamp >= window_start) {
      double abs_error = std::abs(data.error);
      errors.push_back(abs_error);
      sum_error += data.error;
      sum_squared_error += data.error * data.error;
      max_abs_error = std::max(max_abs_error, abs_error);
      
      // Calculate integral (trapezoidal rule)
      if (count > 0) {
        double dt = (data.stamp - last_time).seconds();
        integral_error += abs_error * dt;
      }
      
      last_time = data.stamp;
      count++;
    }
  }
  
  if (count == 0) {
    return stats;
  }
  
  // Calculate statistics
  stats.sample_count = count;
  stats.mean_error = sum_error / count;
  
  if (config_.statistics.enable_rmse) {
    stats.rmse = std::sqrt(sum_squared_error / count);
  }
  
  if (config_.statistics.enable_max_error) {
    stats.max_error = max_abs_error;
  }
  
  if (config_.statistics.enable_integral) {
    stats.integral_error = integral_error;
  }
  
  stats.last_update = now;
  
  return stats;
}

void SteeringErrorMonitor::updateWindow(const ErrorData & data)
{
  // Add new data point
  error_history_.push_back(data);
  
  // Remove old data points outside time window
  rclcpp::Time window_start = node_->now() - rclcpp::Duration::from_seconds(config_.statistics.window_duration);
  
  while (!error_history_.empty() && error_history_.front().stamp < window_start) {
    error_history_.pop_front();
  }
  
  // Limit maximum history size (safety)
  const size_t max_history_size = 10000;
  if (error_history_.size() > max_history_size) {
    error_history_.pop_front();
  }
}

void SteeringErrorMonitor::publishError(double error)
{
  std_msgs::msg::Float64 msg;
  msg.data = error;
  error_pub_->publish(msg);
}

void SteeringErrorMonitor::publishDiagnostics(const ErrorStatistics & stats)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "steering_error_monitor";
  status.hardware_id = "loader_steering_system";
  
  // Determine status level
  if (stats.rmse >= config_.thresholds.critical_rmse) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Critical steering error detected";
  } else if (stats.rmse >= config_.thresholds.error_rmse) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "High steering error detected";
  } else if (stats.rmse >= config_.thresholds.warning_rmse) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Steering error warning";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Steering error within normal range";
  }
  
  // Add values
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "rmse";
  kv.value = std::to_string(stats.rmse);
  status.values.push_back(kv);
  
  kv.key = "max_error";
  kv.value = std::to_string(stats.max_error);
  status.values.push_back(kv);
  
  kv.key = "mean_error";
  kv.value = std::to_string(stats.mean_error);
  status.values.push_back(kv);
  
  kv.key = "integral_error";
  kv.value = std::to_string(stats.integral_error);
  status.values.push_back(kv);
  
  kv.key = "sample_count";
  kv.value = std::to_string(stats.sample_count);
  status.values.push_back(kv);
  
  diagnostics_pub_->publish(status);
}

void SteeringErrorMonitor::publishVisualization(double error)
{
  if (!vis_pub_) {
    return;
  }
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = node_->now();
  marker.ns = "steering_error";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  // Set position (at origin, pointing in steering direction)
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.5;
  marker.pose.orientation.w = 1.0;
  
  // Set scale (arrow length based on error magnitude)
  double arrow_length = std::min(std::abs(error) * 5.0, 1.0);  // Scale factor
  marker.scale.x = arrow_length;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  
  // Set color based on error
  auto color = getErrorColor(error);
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 0.8;
  
  // Set orientation to show error direction
  double yaw = (error > 0) ? M_PI / 2.0 : -M_PI / 2.0;
  marker.pose.orientation.z = std::sin(yaw / 2.0);
  marker.pose.orientation.w = std::cos(yaw / 2.0);
  
  vis_pub_->publish(marker);
}

void SteeringErrorMonitor::logToFile(
  const ErrorData & data,
  const ErrorStatistics & stats)
{
  if (!log_file_.is_open()) {
    return;
  }
  
  // Write CSV row
  int64_t total_nanoseconds = data.stamp.nanoseconds();
  int64_t seconds = total_nanoseconds / 1000000000;
  uint32_t nanoseconds = static_cast<uint32_t>(total_nanoseconds % 1000000000);
  log_file_ << std::fixed << std::setprecision(6)
            << seconds << "." << std::setfill('0') << std::setw(9) << nanoseconds << ","
            << data.delta_cmd << ","
            << data.delta_fb << ","
            << data.error << ","
            << data.velocity_x << ","
            << data.curvature << ","
            << stats.rmse << ","
            << stats.max_error << ","
            << stats.mean_error << ","
            << stats.integral_error << ","
            << stats.sample_count << "\n";
  
  log_file_.flush();
}

void SteeringErrorMonitor::initializeLogFile()
{
  // Create log directory if it doesn't exist
  std::filesystem::path log_dir(config_.logging.log_dir);
  if (!std::filesystem::exists(log_dir)) {
    std::filesystem::create_directories(log_dir);
  }
  
  // Generate filename with timestamp
  std::time_t now = std::time(nullptr);
  std::tm * tm = std::localtime(&now);
  std::ostringstream oss;
  oss << std::put_time(tm, "%Y%m%d_%H%M%S");
  std::string timestamp = oss.str();
  
  std::string filename = config_.logging.log_dir + "/steering_error_" + timestamp + ".csv";
  
  // Open file
  log_file_.open(filename, std::ios::out);
  
  if (log_file_.is_open()) {
    // Write CSV header
    log_file_ << "timestamp,delta_cmd,delta_fb,error,velocity_x,curvature,rmse,max_error,mean_error,integral_error,sample_count\n";
    log_file_.flush();
    log_enabled_ = true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open log file: %s", filename.c_str());
    log_enabled_ = false;
  }
}

std::array<double, 3> SteeringErrorMonitor::getErrorColor(double error) const
{
  double abs_error = std::abs(error);
  
  if (abs_error < config_.thresholds.warning_rmse) {
    return {0.0, 1.0, 0.0};  // Green
  } else if (abs_error < config_.thresholds.error_rmse) {
    return {1.0, 1.0, 0.0};  // Yellow
  } else {
    return {1.0, 0.0, 0.0};  // Red
  }
}

}  // namespace loader_control


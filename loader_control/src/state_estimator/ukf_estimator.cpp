#include "loader_control/state_estimator/ukf_estimator.hpp"
#include <cmath>

namespace loader_control {

void UKFEstimator::configure(double wheelbase, double tau_v, double tau_delta,
                            const std::vector<double> & process_noise,
                            const std::vector<double> & measurement_noise) {
  wheelbase_ = wheelbase;
  tau_v_ = tau_v;
  tau_delta_ = tau_delta;
  process_noise_ = process_noise;
  measurement_noise_ = measurement_noise;

  // 初始化状态均值（5维：x, y, theta, v, delta）
  state_mean_ = std::vector<double>(5, 0.0);
  
  // 初始化协方差矩阵（简化版，实际应为5x5矩阵）
  state_covariance_.clear();
  for (size_t i = 0; i < 5; ++i) {
    state_covariance_.push_back(std::vector<double>(5, 0.0));
    state_covariance_[i][i] = 1.0;  // 初始化为单位矩阵
  }
}

loader_control_interfaces::msg::VehicleState UKFEstimator::estimate(
  const loader_control_interfaces::msg::VehicleState & predicted,
  const loader_control_interfaces::msg::VehicleState & measured) {
  
  // 简化的UKF实现（实际应使用Unscented变换）
  // 这里使用简单的加权融合作为占位符
  
  loader_control_interfaces::msg::VehicleState estimated = measured;
  
  // 简单加权融合（实际应使用完整的UKF算法）
  double alpha = 0.7;  // 融合权重
  estimated.pose.position.x = alpha * predicted.pose.position.x + 
                             (1.0 - alpha) * measured.pose.position.x;
  estimated.pose.position.y = alpha * predicted.pose.position.y + 
                             (1.0 - alpha) * measured.pose.position.y;
  
  // 速度融合
  estimated.velocity = alpha * predicted.velocity + (1.0 - alpha) * measured.velocity;
  
  // 转向角融合
  estimated.steering_angle = alpha * predicted.steering_angle + 
                            (1.0 - alpha) * measured.steering_angle;
  
  return estimated;
}

} // namespace loader_control


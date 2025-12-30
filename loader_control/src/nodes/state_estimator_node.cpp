#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "loader_control/state_estimator/ukf_estimator.hpp"
#include "loader_control_interfaces/msg/vehicle_state.hpp"
#include <vector>

class StateEstimatorNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit StateEstimatorNode(const rclcpp::NodeOptions & options)
  : LifecycleNode("state_estimator", options) {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State &) {
    // 声明参数
    declare_parameter<double>("wheelbase", 2.8);
    declare_parameter<double>("tau_v", 0.8);
    declare_parameter<double>("tau_delta", 0.3);
    declare_parameter<std::vector<double>>("process_noise", 
      std::vector<double>{0.1, 0.1, 0.05, 0.2, 0.1});
    declare_parameter<std::vector<double>>("measurement_noise", 
      std::vector<double>{0.05, 0.05, 0.02, 0.1});

    // 获取参数
    double wheelbase = get_parameter("wheelbase").as_double();
    double tau_v = get_parameter("tau_v").as_double();
    double tau_delta = get_parameter("tau_delta").as_double();
    auto process_noise = get_parameter("process_noise").as_double_array();
    auto measurement_noise = get_parameter("measurement_noise").as_double_array();

    // 配置估计器
    std::vector<double> proc_noise_vec(process_noise.begin(), process_noise.end());
    std::vector<double> meas_noise_vec(measurement_noise.begin(), measurement_noise.end());
    estimator_.configure(wheelbase, tau_v, tau_delta, proc_noise_vec, meas_noise_vec);

    // 创建发布/订阅（这里简化，实际应从多个传感器订阅）
    estimated_state_pub_ = create_publisher<loader_control_interfaces::msg::VehicleState>(
      "/localization/vehicle_state", 10);

    RCLCPP_INFO(this->get_logger(), "State estimator configured");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_activate(const rclcpp_lifecycle::State &) {
    estimated_state_pub_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_deactivate(const rclcpp_lifecycle::State &) {
    estimated_state_pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<loader_control_interfaces::msg::VehicleState>::SharedPtr estimated_state_pub_;
  loader_control::UKFEstimator estimator_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(StateEstimatorNode)


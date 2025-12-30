#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "loader_control_interfaces/msg/vehicle_state.hpp"
#include "loader_control/utils/safety_monitor.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class ControlManagerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit ControlManagerNode(const rclcpp::NodeOptions & options)
  : LifecycleNode("control_manager", options) {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State &) {
    // 订阅关键状态
    state_sub_ = create_subscription<loader_control_interfaces::msg::VehicleState>(
      "/localization/vehicle_state", 10,
      std::bind(&ControlManagerNode::stateCallback, this, std::placeholders::_1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planning/local_trajectory", 10,
      std::bind(&ControlManagerNode::pathCallback, this, std::placeholders::_1));

    // 提供服务
    emergency_stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/control/emergency_stop",
      std::bind(&ControlManagerNode::emergencyStopCallback, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // 从参数服务器读取安全阈值
    declare_parameter<double>("max_lateral_error", 1.5);
    declare_parameter<double>("max_heading_error", 1.0);
    declare_parameter<double>("emergency_stop_timeout", 2.0);
    
    max_lat_err_ = get_parameter("max_lateral_error").as_double();
    max_heading_err_ = get_parameter("max_heading_error").as_double();
    emergency_stop_timeout_ = get_parameter("emergency_stop_timeout").as_double();

    safety_monitor_.setMaxLateralError(max_lat_err_);
    safety_monitor_.setMaxHeadingError(max_heading_err_);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_activate(const rclcpp_lifecycle::State &) {
    monitor_timer_ = create_wall_timer(100ms, std::bind(&ControlManagerNode::monitorLoop, this));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_deactivate(const rclcpp_lifecycle::State &) {
    monitor_timer_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void stateCallback(const loader_control_interfaces::msg::VehicleState::SharedPtr msg) {
    current_state_ = *msg;
    last_state_time_ = this->now();
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    reference_path_ = *msg;
  }

  void monitorLoop() {
    if (reference_path_.poses.empty() || !current_state_.header.stamp.sec) {
      return;
    }

    // 检查安全性
    bool is_safe = safety_monitor_.checkSafety(current_state_, reference_path_);
    
    if (!is_safe) {
      double lat_err = safety_monitor_.getLateralError();
      double heading_err = safety_monitor_.getHeadingError();
      RCLCPP_WARN(this->get_logger(), 
                  "Safety violation! Lateral error: %.2f, Heading error: %.2f", 
                  lat_err, heading_err);
      // 这里可以添加降速或停止逻辑
    }

    // 检查超时
    auto elapsed = (this->now() - last_state_time_).seconds();
    if (elapsed > emergency_stop_timeout_) {
      RCLCPP_ERROR(this->get_logger(), "State update timeout! Elapsed: %.2f s", elapsed);
    }
  }

  void emergencyStopCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    RCLCPP_WARN(this->get_logger(), "Emergency stop triggered!");
    
    // 这里可以添加停止逻辑，例如发布零速度指令
    // 或通过服务调用让其他节点停止
    
    response->success = true;
    response->message = "Emergency stop triggered";
  }

  rclcpp::Subscription<loader_control_interfaces::msg::VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  loader_control_interfaces::msg::VehicleState current_state_;
  nav_msgs::msg::Path reference_path_;
  
  double max_lat_err_{1.5};
  double max_heading_err_{1.0};
  double emergency_stop_timeout_{2.0};
  
  rclcpp::Time last_state_time_;
  loader_control::SafetyMonitor safety_monitor_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(ControlManagerNode)


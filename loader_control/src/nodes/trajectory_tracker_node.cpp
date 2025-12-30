#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "loader_control/controllers/base_controller.hpp"
#include "loader_control/controllers/pid_controller.hpp"
#include "loader_control/controllers/pure_pursuit_controller.hpp"
#include "loader_control/controllers/mpc_controller.hpp"
#include "loader_control/controllers/smc_controller.hpp"
#include "loader_control/utils/trajectory_interpolator.hpp"

#include "loader_control_interfaces/msg/raw_control_cmd.hpp"
#include "loader_control_interfaces/msg/vehicle_state.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class TrajectoryTrackerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit TrajectoryTrackerNode(const rclcpp::NodeOptions & options)
  : LifecycleNode("trajectory_tracker", options) {}

  // ===== Lifecycle Callbacks =====
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State &) {
    // 1. 声明参数
    declare_parameter<std::string>("controller.type", "pure_pursuit");
    declare_parameter<double>("control_rate", 20.0);
    
    // 2. 创建发布/订阅
    raw_cmd_pub_ = create_publisher<loader_control_interfaces::msg::RawControlCmd>(
      "/control/raw_command", 10);
    state_sub_ = create_subscription<loader_control_interfaces::msg::VehicleState>(
      "/localization/vehicle_state", 10,
      std::bind(&TrajectoryTrackerNode::stateCallback, this, std::placeholders::_1));
    traj_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planning/local_trajectory", 10,
      std::bind(&TrajectoryTrackerNode::trajectoryCallback, this, std::placeholders::_1));

    // 3. 加载控制器
    std::string ctrl_type = get_parameter("controller.type").as_string();
    if (ctrl_type == "pid") {
      controller_ = std::make_unique<loader_control::PIDController>();
    } else if (ctrl_type == "pure_pursuit") {
      controller_ = std::make_unique<loader_control::PurePursuitController>();
    } else if (ctrl_type == "mpc") {
      controller_ = std::make_unique<loader_control::MPCController>();
    } else if (ctrl_type == "smc") {
      controller_ = std::make_unique<loader_control::SMCController>();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown controller: %s", ctrl_type.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    controller_->configure(shared_from_this());

    RCLCPP_INFO(this->get_logger(), "Configured with controller: %s", ctrl_type.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_activate(const rclcpp_lifecycle::State &) {
    raw_cmd_pub_->on_activate();
    double rate = get_parameter("control_rate").as_double();
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    timer_ = create_wall_timer(
      period,
      std::bind(&TrajectoryTrackerNode::controlLoop, this));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_deactivate(const rclcpp_lifecycle::State &) {
    timer_.reset();
    raw_cmd_pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void stateCallback(const loader_control_interfaces::msg::VehicleState::SharedPtr msg) {
    current_state_ = *msg;
    state_updated_ = true;
  }

  void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    reference_path_ = *msg;
    traj_updated_ = true;
  }

  void controlLoop() {
    if (!state_updated_ || !traj_updated_ || reference_path_.poses.empty()) return;

    // 构建输入
    loader_control::ControllerInput input;
    input.current_pose.header = current_state_.header;
    input.current_pose.pose = current_state_.pose;
    input.current_speed = current_state_.velocity;
    input.current_steering = current_state_.steering_angle;
    input.reference_path = reference_path_;
    input.closest_idx = loader_control::TrajectoryInterpolator::findNearestIndex(
      reference_path_, input.current_pose.pose);

    // 计算控制量
    auto output = controller_->compute(input);

    // 发布原始指令
    auto cmd = loader_control_interfaces::msg::RawControlCmd();
    cmd.header.stamp = this->now();
    cmd.throttle = output.throttle_cmd;
    cmd.steering = output.steering_cmd;
    raw_cmd_pub_->publish(cmd);

    // 重置标志
    state_updated_ = false;
    traj_updated_ = false;
  }

  // 成员变量
  rclcpp_lifecycle::LifecyclePublisher<loader_control_interfaces::msg::RawControlCmd>::SharedPtr raw_cmd_pub_;
  rclcpp::Subscription<loader_control_interfaces::msg::VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  loader_control_interfaces::msg::VehicleState current_state_;
  nav_msgs::msg::Path reference_path_;
  bool state_updated_{false};
  bool traj_updated_{false};

  std::unique_ptr<loader_control::BaseController> controller_;
};

// ===== 注册节点 =====
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(TrajectoryTrackerNode)


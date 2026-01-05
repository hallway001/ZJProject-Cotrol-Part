#include "loader_control/control_node.hpp"
#include <memory>
#include <chrono>
#include <cmath>

namespace loader_control
{

ControlNode::ControlNode(const std::string & node_name)
: Node(node_name),
  current_controller_type_("pid"),
  control_rate_(50.0),
  use_actuator_models_(true),
  enable_error_monitor_(true),
  enable_visualization_(true),
  state_initialized_(false)
{
  // Declare parameters
  declare_parameter("controller_type", "pid");
  declare_parameter("control_rate", 50.0);
  declare_parameter("use_actuator_models", true);
  declare_parameter("enable_error_monitor", true);
  declare_parameter("enable_visualization", true);
  declare_parameter("vehicle_model", "loader_v1");
  
  // Get parameters
  current_controller_type_ = get_parameter("controller_type").as_string();
  control_rate_ = get_parameter("control_rate").as_double();
  use_actuator_models_ = get_parameter("use_actuator_models").as_bool();
  enable_error_monitor_ = get_parameter("enable_error_monitor").as_bool();
  enable_visualization_ = get_parameter("enable_visualization").as_bool();
  std::string vehicle_model = get_parameter("vehicle_model").as_string();
  
  RCLCPP_INFO(get_logger(), "Initializing Loader Control Node");
  RCLCPP_INFO(get_logger(), "Controller type: %s", current_controller_type_.c_str());
  RCLCPP_INFO(get_logger(), "Vehicle model: %s", vehicle_model.c_str());
  RCLCPP_INFO(get_logger(), "Control rate: %.1f Hz", control_rate_);
  
  // Create data receiver (separate node)
  data_receiver_ = std::make_shared<DataReceiver>("data_receiver");
  
  // Initialize actuators
  initializeActuators();
  
  // Initialize controller
  initializeController(current_controller_type_);
  
  // Initialize error monitor
  if (enable_error_monitor_) {
    initializeErrorMonitor();
  }
  
  // Create publishers
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/loader/cmd_vel", 10);
  if (enable_visualization_) {
    vis_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/loader/control/visualization", 10);
  }
  
  // Register parameter callback
  [[maybe_unused]] auto param_callback_handle = add_on_set_parameters_callback(
    std::bind(&ControlNode::onParameterChange, this, std::placeholders::_1));
  
  // Create control loop timer
  double period = 1.0 / control_rate_;
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(period * 1000)),
    std::bind(&ControlNode::controlLoop, this));
  
  RCLCPP_INFO(get_logger(), "Loader Control Node initialized successfully");
}

ControlNode::~ControlNode()
{
}

void ControlNode::initializeActuators()
{
  // Load actuator parameters from parameter server
  ActuatorModel::Config steering_config;
  steering_config.delay_time_constant = declare_parameter(
    "actuator.steering.delay_time_constant", 0.15);
  steering_config.dead_zone = declare_parameter(
    "actuator.steering.dead_zone", 0.02);
  steering_config.scale_bias = declare_parameter(
    "actuator.steering.scale_bias", 1.02);
  steering_config.noise_std = declare_parameter(
    "actuator.steering.noise_std", 0.005);
  steering_config.max_value = declare_parameter(
    "actuator.steering.max_angle", 0.785);
  steering_config.min_value = declare_parameter(
    "actuator.steering.min_angle", -0.785);
  steering_config.filter_alpha = declare_parameter(
    "actuator.steering.filter_alpha", 0.8);
  
  ActuatorModel::Config throttle_config;
  throttle_config.delay_time_constant = declare_parameter(
    "actuator.throttle.delay_time_constant", 0.2);
  throttle_config.dead_zone = declare_parameter(
    "actuator.throttle.dead_zone", 0.05);
  throttle_config.scale_bias = declare_parameter(
    "actuator.throttle.scale_bias", 1.0);
  throttle_config.noise_std = declare_parameter(
    "actuator.throttle.noise_std", 0.0);
  throttle_config.max_value = declare_parameter(
    "actuator.throttle.max_value", 1.0);
  throttle_config.min_value = declare_parameter(
    "actuator.throttle.min_value", 0.0);
  throttle_config.filter_alpha = declare_parameter(
    "actuator.throttle.filter_alpha", 0.7);
  
  if (use_actuator_models_) {
    steering_actuator_ = std::make_shared<ActuatorModel>(steering_config, "steering");
    throttle_actuator_ = std::make_shared<ActuatorModel>(throttle_config, "throttle");
    
    RCLCPP_INFO(get_logger(), "Actuator models initialized");
  }
}

void ControlNode::initializeController(const std::string & controller_type)
{
  if (controller_type == "pid") {
    // Initialize PID controller
    PIDController::Config pid_config;
    pid_config.update_rate = control_rate_;
    
    pid_config.longitudinal.enable = declare_parameter(
      "pid_controller.longitudinal.enable", true);
    pid_config.longitudinal.kp = declare_parameter(
      "pid_controller.longitudinal.kp", 1.5);
    pid_config.longitudinal.ki = declare_parameter(
      "pid_controller.longitudinal.ki", 0.1);
    pid_config.longitudinal.kd = declare_parameter(
      "pid_controller.longitudinal.kd", 0.3);
    pid_config.longitudinal.max_integral = declare_parameter(
      "pid_controller.longitudinal.max_integral", 1.0);
    
    pid_config.lateral.enable = declare_parameter(
      "pid_controller.lateral.enable", true);
    pid_config.lateral.kp = declare_parameter(
      "pid_controller.lateral.kp", 2.0);
    pid_config.lateral.ki = declare_parameter(
      "pid_controller.lateral.ki", 0.05);
    pid_config.lateral.kd = declare_parameter(
      "pid_controller.lateral.kd", 0.5);
    pid_config.lateral.max_integral = declare_parameter(
      "pid_controller.lateral.max_integral", 0.5);
    
    pid_config.steering_compensation.enable = declare_parameter(
      "pid_controller.steering_compensation.enable", true);
    pid_config.steering_compensation.ff_gain = declare_parameter(
      "pid_controller.steering_compensation.ff_gain", 0.3);
    
    controller_ = std::make_shared<PIDController>(pid_config);
    
  } else if (controller_type == "pure_pursuit" || controller_type == "purepursuit") {
    // Initialize Pure Pursuit controller
    PurePursuitController::Config pp_config;
    pp_config.update_rate = control_rate_;
    
    pp_config.lookahead.base_distance = declare_parameter(
      "pure_pursuit_controller.lookahead.base_distance", 2.0);
    pp_config.lookahead.min_distance = declare_parameter(
      "pure_pursuit_controller.lookahead.min_distance", 1.0);
    pp_config.lookahead.max_distance = declare_parameter(
      "pure_pursuit_controller.lookahead.max_distance", 5.0);
    pp_config.lookahead.velocity_gain = declare_parameter(
      "pure_pursuit_controller.lookahead.velocity_gain", 0.5);
    
    pp_config.error_compensation.enable = declare_parameter(
      "pure_pursuit_controller.error_compensation.enable", true);
    pp_config.error_compensation.lookahead_error_gain = declare_parameter(
      "pure_pursuit_controller.error_compensation.lookahead_error_gain", 0.5);
    pp_config.error_compensation.max_error_compensation = declare_parameter(
      "pure_pursuit_controller.error_compensation.max_error_compensation", 1.0);
    
    controller_ = std::make_shared<PurePursuitController>(pp_config);
    
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown controller type: %s", controller_type.c_str());
    return;
  }
  
  // Set actuator models
  if (use_actuator_models_) {
    controller_->setActuatorModels(steering_actuator_, throttle_actuator_);
  }
  
  // Initialize controller
  if (!controller_->initialize()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize controller");
    return;
  }
  
  RCLCPP_INFO(get_logger(), "Controller initialized: %s", controller_type.c_str());
}

void ControlNode::initializeErrorMonitor()
{
  SteeringErrorMonitor::Config monitor_config;
  monitor_config.update_rate = control_rate_;
  
  monitor_config.statistics.window_duration = declare_parameter(
    "error_monitor.statistics.window_duration", 2.0);
  monitor_config.statistics.enable_rmse = declare_parameter(
    "error_monitor.statistics.enable_rmse", true);
  
  monitor_config.thresholds.warning_rmse = declare_parameter(
    "error_monitor.thresholds.warning_rmse", 0.03);
  monitor_config.thresholds.error_rmse = declare_parameter(
    "error_monitor.thresholds.error_rmse", 0.05);
  monitor_config.thresholds.critical_rmse = declare_parameter(
    "error_monitor.thresholds.critical_rmse", 0.1);
  
  monitor_config.logging.enable = declare_parameter(
    "error_monitor.logging.enable", true);
  monitor_config.logging.log_dir = declare_parameter(
    "error_monitor.logging.log_dir", std::string("logs"));
  
  monitor_config.visualization.enable = declare_parameter(
    "error_monitor.visualization.enable", true);
  
  error_monitor_ = std::make_shared<SteeringErrorMonitor>(
    shared_from_this(), monitor_config);
  
  RCLCPP_INFO(get_logger(), "Error monitor initialized");
}

void ControlNode::controlLoop()
{
  // Get current vehicle state
  VehicleState state = data_receiver_->getVehicleState();
  
  if (!data_receiver_->isStateValid() || state.ref_path.poses.empty()) {
    // State not valid or no path available
    return;
  }
  
  // Calculate time step
  rclcpp::Time now = this->now();
  double dt = 0.02;  // Default 20ms
  if (state_initialized_) {
    dt = (now - last_state_.stamp).seconds();
    if (dt <= 0.0 || dt > 0.1) {  // Sanity check
      dt = 1.0 / control_rate_;
    }
  }
  
  // Compute control command
  ControlCommand cmd = controller_->computeCommand(state, dt);
  cmd.stamp = now;
  
  // Update state with control commands for error calculation
  state.delta_cmd = cmd.steering_command;
  state.updateSteeringError();
  
  // Update error monitor
  if (enable_error_monitor_ && error_monitor_) {
    error_monitor_->update(state);
  }
  
  // Publish command
  publishCommand(cmd);
  
  // Publish visualization
  if (enable_visualization_) {
    publishVisualization(state, cmd);
  }
  
  // Update last state
  last_state_ = state;
  last_command_ = cmd;
  state_initialized_ = true;
}

rcl_interfaces::msg::SetParametersResult ControlNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto & param : parameters) {
    if (param.get_name() == "controller_type") {
      std::string new_type = param.as_string();
      if (new_type != current_controller_type_) {
        RCLCPP_INFO(get_logger(), "Switching controller from %s to %s",
                    current_controller_type_.c_str(), new_type.c_str());
        initializeController(new_type);
        current_controller_type_ = new_type;
      }
    }
  }
  
  return result;
}

void ControlNode::publishCommand(const ControlCommand & cmd)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = cmd.velocity_command;
  msg.angular.z = cmd.steering_command;
  cmd_vel_pub_->publish(msg);
}

void ControlNode::publishVisualization(const VehicleState & state, const ControlCommand & /* cmd */)
{
  // Publish reference path
  if (!state.ref_path.poses.empty()) {
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = now();
    path_marker.ns = "reference_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x = 0.1;
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;
    
    for (const auto & pose_stamped : state.ref_path.poses) {
      geometry_msgs::msg::Point point;
      point.x = pose_stamped.pose.position.x;
      point.y = pose_stamped.pose.position.y;
      point.z = 0.0;
      path_marker.points.push_back(point);
    }
    
    vis_marker_pub_->publish(path_marker);
  }
  
  // Publish vehicle position
  visualization_msgs::msg::Marker vehicle_marker;
  vehicle_marker.header.frame_id = "map";
  vehicle_marker.header.stamp = now();
  vehicle_marker.ns = "vehicle";
  vehicle_marker.id = 1;
  vehicle_marker.type = visualization_msgs::msg::Marker::ARROW;
  vehicle_marker.action = visualization_msgs::msg::Marker::ADD;
  vehicle_marker.pose.position.x = state.x;
  vehicle_marker.pose.position.y = state.y;
  vehicle_marker.pose.position.z = 0.0;
  vehicle_marker.pose.orientation.z = std::sin(state.yaw / 2.0);
  vehicle_marker.pose.orientation.w = std::cos(state.yaw / 2.0);
  vehicle_marker.scale.x = 1.0;
  vehicle_marker.scale.y = 0.3;
  vehicle_marker.scale.z = 0.3;
  vehicle_marker.color.r = 0.0;
  vehicle_marker.color.g = 0.0;
  vehicle_marker.color.b = 1.0;
  vehicle_marker.color.a = 1.0;
  
  vis_marker_pub_->publish(vehicle_marker);
}

}  // namespace loader_control

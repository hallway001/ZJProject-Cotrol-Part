#ifndef LOADER_CONTROL__CONTROL_NODE_HPP_
#define LOADER_CONTROL__CONTROL_NODE_HPP_

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64.hpp>
#include <yaml-cpp/yaml.h>
#include "loader_control/data_receiver.hpp"
#include "loader_control/controller_base.hpp"
#include "loader_control/pid_controller.hpp"
#include "loader_control/pure_pursuit_controller.hpp"
#include "loader_control/actuator_model.hpp"
#include "loader_control/steering_error_monitor.hpp"
#include "loader_control/vehicle_state.hpp"

namespace loader_control
{

/**
 * @brief Main control node for loader autonomous driving
 * Integrates sensor data, controllers, actuator models, and error monitoring
 */
class ControlNode : public rclcpp::Node
{
public:
  explicit ControlNode(const std::string & node_name = "loader_control_node");
  
  ~ControlNode();

protected:
  /**
   * @brief Load configuration from YAML files
   */
  bool loadConfig(const std::string & config_path);
  
  /**
   * @brief Merge YAML configurations (base + override)
   */
  YAML::Node mergeConfigs(const YAML::Node & base, const YAML::Node & override);
  
  /**
   * @brief Initialize actuators
   */
  void initializeActuators();
  
  /**
   * @brief Initialize controller
   */
  void initializeController(const std::string & controller_type);
  
  /**
   * @brief Initialize error monitor
   */
  void initializeErrorMonitor();
  
  /**
   * @brief Main control loop
   */
  void controlLoop();
  
  /**
   * @brief Parameter change callback
   */
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters);
  
  /**
   * @brief Publish control command
   */
  void publishCommand(const ControlCommand & cmd);
  
  /**
   * @brief Publish visualization
   */
  void publishVisualization(const VehicleState & state, const ControlCommand & cmd);

private:
  // Data receiver
  std::shared_ptr<DataReceiver> data_receiver_;
  
  // Controller (plugin-style, can switch dynamically)
  std::shared_ptr<ControllerBase> controller_;
  std::string current_controller_type_;
  
  // Actuator models (for simulation/testing)
  std::shared_ptr<ActuatorModel> steering_actuator_;
  std::shared_ptr<ActuatorModel> throttle_actuator_;
  
  // Error monitor
  std::shared_ptr<SteeringErrorMonitor> error_monitor_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_marker_pub_;
  
  // Timer for control loop
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // Configuration
  YAML::Node config_;
  
  // Control parameters
  double control_rate_ = 50.0;  // Hz
  bool use_actuator_models_ = true;
  bool enable_error_monitor_ = true;
  bool enable_visualization_ = true;
  
  // Last control command
  ControlCommand last_command_;
  
  // State tracking
  VehicleState last_state_;
  bool state_initialized_ = false;
};

}  // namespace loader_control

#endif  // LOADER_CONTROL__CONTROL_NODE_HPP_


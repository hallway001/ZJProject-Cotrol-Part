#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "loader_control/utils/command_modulator.hpp"
#include "loader_control_interfaces/msg/raw_control_cmd.hpp"
#include "loader_control_interfaces/msg/actuator_cmd.hpp"

class CommandModulatorNode : public rclcpp::Node {
public:
  CommandModulatorNode() : Node("command_modulator") {
    modulator_ = std::make_unique<loader_control::CommandModulator>(shared_from_this());

    raw_sub_ = create_subscription<loader_control_interfaces::msg::RawControlCmd>(
      "/control/raw_command", 10,
      std::bind(&CommandModulatorNode::rawCmdCallback, this, std::placeholders::_1));
    
    actuator_pub_ = create_publisher<loader_control_interfaces::msg::ActuatorCmd>(
      "/control/actuator_command", 10);
  }

private:
  void rawCmdCallback(const loader_control_interfaces::msg::RawControlCmd::SharedPtr msg) {
    if (!last_time_) {
      last_time_ = msg->header.stamp;
      return;
    }

    double dt = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(*last_time_)).seconds();
    last_time_ = msg->header.stamp;

    auto modulated = modulator_->modulate(*msg, dt);
    modulated.header = msg->header;
    actuator_pub_->publish(modulated);
  }

  rclcpp::Subscription<loader_control_interfaces::msg::RawControlCmd>::SharedPtr raw_sub_;
  rclcpp::Publisher<loader_control_interfaces::msg::ActuatorCmd>::SharedPtr actuator_pub_;
  std::unique_ptr<loader_control::CommandModulator> modulator_;
  std::optional<builtin_interfaces::msg::Time> last_time_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(CommandModulatorNode)


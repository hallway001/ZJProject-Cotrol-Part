#include <rclcpp/rclcpp.hpp>
#include "loader_control/control_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<loader_control::ControlNode>("loader_control_node");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}


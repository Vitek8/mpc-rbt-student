#include "../include/KeyboardControl.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto keyboard_control_node = std::make_shared<KeyboardControlNode>();
  rclcpp::spin(keyboard_control_node);
  rclcpp::shutdown();
  return 0;
}

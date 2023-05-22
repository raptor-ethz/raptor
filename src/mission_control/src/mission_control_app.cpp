#include "rclcpp/rclcpp.hpp"

#include "mission_control/mission_control.hpp"




int main(int argc, char *argv[])
{
  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto mission_control_node = std::make_shared<MissionControl>();

  // spin node if initialization was successful
  if(rclcpp::ok()) {
    RCLCPP_INFO(mission_control_node->get_logger(), "Ready.");
    rclcpp::spin(mission_control_node);
  }

  rclcpp::shutdown();
  return 0;
}
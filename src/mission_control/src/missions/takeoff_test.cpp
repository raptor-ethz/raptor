#include "rclcpp/rclcpp.hpp"

#include "mission_control/mission_control.hpp"




int main(int argc, char *argv[])
{
  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto mission_control_node = std::make_shared<MissionControl>();

  // spin node if initialization was successful
  if(!rclcpp::ok()) {
    rclcpp::shutdown();
    return 0;
  }

  RCLCPP_INFO(mission_control_node->get_logger(), "Ready.");
  mission_control_node->arm();
  rclcpp::spin_some(mission_control_node);

  if(!rclcpp::ok()) {
    rclcpp::shutdown();
    return 0;
  }

  mission_control_node->takeoff(0.7);
  rclcpp::spin_some(mission_control_node);

  rclcpp::shutdown();
  return 0;
}
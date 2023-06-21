#include "rclcpp/rclcpp.hpp"

#include "mission_control/mission_control.hpp"




std::array<float, 3> home_position = {0.0, 0.0, 0.7};
std::array<float, 3> home_position_low = {0.0, 0.0, 0.4};

std::array<float, 3> start_position = {-0.5, -1.0, 1.5};
std::array<float, 3> swoop_position = {1.0, -1.0, 0.7};
std::array<float, 3> end_position = {2.5, -1.0, 1.5};


int main(int argc, char *argv[])
{
  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto mission_control_node = std::make_shared<MissionControl>();
  if(!rclcpp::ok()) {mission_control_node->shutdown();}
  RCLCPP_INFO(mission_control_node->get_logger(), "Starting mission.");

  // takeoff
  if(!mission_control_node->arm()) {mission_control_node->shutdown();}
  if(!mission_control_node->takeoff(0.7)) {mission_control_node->shutdown();}

  // mission
  if(!mission_control_node->go_to_pos(start_position, 0.0, 7.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_pos(swoop_position, 0.0, 2.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_pos(end_position, 0.0, 7.0, true)) {mission_control_node->shutdown();}

  // land
  if(!mission_control_node->go_to_pos(home_position, 0.0, 4.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_pos(home_position_low, 0.0, 2.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->land()) {mission_control_node->shutdown();}

  RCLCPP_INFO(mission_control_node->get_logger(), "Mission complete. Shutting down.");
  rclcpp::shutdown();
  return 0;
}
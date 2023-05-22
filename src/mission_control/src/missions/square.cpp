#include "rclcpp/rclcpp.hpp"

#include "mission_control/mission_control.hpp"




std::array<float, 3> edge_0 = {1.0, 0.0, 0.7};
std::array<float, 3> edge_1 = {1.0, -1.0, 0.7};
std::array<float, 3> edge_2 = {0.0, -1.0, 0.7};
std::array<float, 3> edge_3 = {0.0, 0.0, 0.7};

std::array<std::array<float, 3>, 4> square_waypoints = {
  edge_0,
  edge_1,
  edge_2,
  edge_3
};


int main(int argc, char *argv[])
{
  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto mission_control_node = std::make_shared<MissionControl>();

  if(!rclcpp::ok()) {mission_control_node->shutdown();}

  RCLCPP_INFO(mission_control_node->get_logger(), "Starting mission.");

  if(!mission_control_node->arm()) {mission_control_node->shutdown();}

  if(!rclcpp::ok()) {mission_control_node->shutdown(); }

  if(!mission_control_node->takeoff(0.7)) {mission_control_node->shutdown();}

  
  // fly the square
  for (size_t i = 0; i < square_waypoints.size(); i++) {
    if(!mission_control_node->go_to_pos(square_waypoints[i], 0.0, 4.0, true)) {mission_control_node->shutdown();}
  }


  if(!mission_control_node->land()) {mission_control_node->shutdown();}

  RCLCPP_INFO(mission_control_node->get_logger(), "Mission complete. Shutting down.");

  rclcpp::shutdown();
  return 0;
}
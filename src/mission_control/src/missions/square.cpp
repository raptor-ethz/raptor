#include "rclcpp/rclcpp.hpp"

#include "mission_control/mission_control.hpp"




std::array<float, 3> home_position = {0.0, 0.0, 0.7};
std::array<float, 3> home_position_low = {0.0, 0.0, 0.4};

// small square
std::array<float, 3> edge_1 = {-0.5, -0.5, 0.7};
std::array<float, 3> edge_2 = {-0.5, 0.5, 0.7};
std::array<float, 3> edge_3 = {0.5, 0.5, 0.7};
std::array<float, 3> edge_4 = {0.5, -0.5, 0.7};

// big square
std::array<float, 3> edge_5 = {-0.5, -2.0, 1.0};
std::array<float, 3> edge_6 = {1.5, -2.0, 1.0};
std::array<float, 3> edge_7 = {1.5, 0.0, 1.0};
std::array<float, 3> edge_8 = {-0.5, 0.0, 1.0};

// 3d square
std::array<float, 3> edge_9 = {-0.5, -2.0, 0.7};
std::array<float, 3> edge_10 = {1.5, -2.0, 2.0};
std::array<float, 3> edge_11 = {1.5, 0.0, 1.5};
std::array<float, 3> edge_12 = {-0.5, 0.0, 1.0};


std::array<std::array<float, 3>, 5> small_square = {
  edge_1,
  edge_2,
  edge_3,
  edge_4,
  edge_1
};

std::array<std::array<float, 3>, 5> big_square = {
  edge_5,
  edge_6,
  edge_7,
  edge_8,
  edge_5
};

std::array<std::array<float, 3>, 5> big_square_3d = {
  edge_9,
  edge_10,
  edge_11,
  edge_12,
  edge_9
};


std::array<std::array<float, 3>, 9> small_big_square_3d = {
  // small
  edge_1,
  edge_2,
  edge_3,
  edge_4,
  edge_1,
  // big
  // edge_9,
  edge_10,
  edge_11,
  edge_12,
  edge_9
};



auto waypoints = small_square; // choose which square to fly


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
  for (size_t i = 0; i < waypoints.size(); i++) {
    if(!mission_control_node->go_to_pos(waypoints[i], 0.0, 4.0, true)) {mission_control_node->shutdown();}
  }

  // land
  if(!mission_control_node->go_to_pos(home_position, 0.0, 4.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->go_to_pos(home_position_low, 0.0, 2.0, true)) {mission_control_node->shutdown();}
  if(!mission_control_node->land()) {mission_control_node->shutdown();}

  RCLCPP_INFO(mission_control_node->get_logger(), "Mission complete. Shutting down.");
  rclcpp::shutdown();
  return 0;
}
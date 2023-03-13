#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

// mavsdk
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// Quad library
#include "quad_control/quad.hpp"

int main(int argc, char *argv[])
{
  // initialize ros
  rclcpp::init(argc, argv);

  // check input
  if (argc < 2)
  {
    std::cerr << "Port not provided.\n";
    usage(argv[0]);
    return 1;
  }

  // start node
  auto quad_control_node = std::make_shared<Quad>();
  // if ( !(quad_control_node->initialize(argv[1])) ) {
  //   rclcpp::shutdown();
  //   return 1;
  // }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Quad Control Node is ready.");

  rclcpp::spin(quad_control_node);
  rclcpp::shutdown();
  return 0;
}
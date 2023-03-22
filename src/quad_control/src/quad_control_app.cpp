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
  // check command line input
  if (argc < 2)
  {
    std::cerr << "Port not provided.\n";
    usage(argv[0]);
    return 1;
  }

  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto quad_control_node = std::make_shared<Quad>(argv[1]);

  // spin node if initialization was successful
  if(rclcpp::ok()) {
    RCLCPP_INFO(quad_control_node->get_logger(), "Ready.");
    rclcpp::spin(quad_control_node);
  }

  rclcpp::shutdown();
  return 0;
}
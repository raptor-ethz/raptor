#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"       // ros2 client library
// #include "std_msgs/msg/string.hpp" // include std_msg type
// #include "std_srvs/srv/trigger.hpp"

// #include "geometry_msgs/msg/point.hpp"

// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// Quad library
#include "quad.hpp"

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

  // start service node
  auto interface = std::make_shared<Quad>();
  if ( !(interface->initialize(argv[1])) ) {
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MavSDK interface server is ready.");

  // spin node
  rclcpp::spin(interface);
  rclcpp::shutdown();
  return 0;
}
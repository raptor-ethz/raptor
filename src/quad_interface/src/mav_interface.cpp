#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"       // ros2 client library
#include "std_msgs/msg/string.hpp" // include std_msg type
#include "std_srvs/srv/trigger.hpp"

#include "geometry_msgs/msg/point.hpp"

// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// helpers
#include "mavsdk/mavsdk_helper.h"

// Quad library
#include "quad.hpp"

int main(int argc, char *argv[])
{
  /* INITIALIZE ROS*/
  rclcpp::init(argc, argv);

  /* INITIALIZE MAVSDK*/
  if (argc != 2)
  {
    usage(argv[0]);
    return 1;
  }

  mavsdk::Mavsdk mavsdk;
  mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

  if (connection_result != mavsdk::ConnectionResult::Success)
  {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }

  auto system = get_system(mavsdk);
  if (!system)
  {
    return 1;
  }

  auto action = mavsdk::Action{system};       // for arming / disarming etc
  auto offboard = mavsdk::Offboard{system};   // for offboard control
  auto telemetry = mavsdk::Telemetry{system}; // for telemetry services

  // start service node
  auto node = std::make_shared<Quad>(&action, &offboard, &telemetry);

  // spin node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
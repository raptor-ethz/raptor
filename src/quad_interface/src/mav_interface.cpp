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

// class MinimalPublisher : public rclcpp::Node
// {
// public:
//   MinimalPublisher()
//       : Node("minimal_publisher"), count_(0)
//   {
//     publisher_ = this->create_publisher<geometry_msgs::msg::Point>("position", 10);
//     timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(500), std::bind(&MinimalPublisher::timer_callback, this));
//   }

// private:
//   void timer_callback()
//   {
//     auto message = geometry_msgs::msg::Point();
//     message.x = 1.0;
//     message.y = 2.5;
//     message.z = -2.1;
//     RCLCPP_INFO(this->get_logger(), "Publishing: [%f,%f,%f]", message.x, message.y, message.z);
//     publisher_->publish(message);
//   }
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
//   size_t count_;
// };

int main(int argc, char *argv[])
{
  /* INITIALIZE ROS*/
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<MinimalPublisher>());
  // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("quad_interface");

  // init mavsdk
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

  // auto test_node = std::make_shared<MinimalPublisher>();

  // rclcpp::spin(test_node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp" // ros2 client library
#include "std_msgs/msg/string.hpp" // include std_msg type

// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// helpers
#include "mavsdk/mavsdk_helper.h"

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{ 
  /* INITIALIZE ROS*/
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();

    /* INITIALIZE MAVSDK */
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

  auto action = mavsdk::Action{system};              // for arming / disarming etc
  auto offboard = mavsdk::Offboard{system};          // for offboard control
  auto telemetry = mavsdk::Telemetry{system};        // for telemetry services

  /* ARM QUADCOPTER */
  const auto arm_result = action.arm();

  std::cout << "Initialize program. \n";


  return 0;
}
#pragma once

// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"       // ros2 client library
#include "std_msgs/msg/string.hpp" // include std_msg type
// std interface
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/point.hpp"
// #include "rclcpp_action/rclcpp_action.hpp" // TODO
// custom interface
#include "raptor_interface/srv/go_to_pos.hpp"
#include "raptor_interface/srv/quad_status.hpp"


class Quad : public rclcpp::Node
{
public:
    Quad();
    ~Quad();

private:
  // state
  bool initialized_{false};
  bool is_airborne_{false};
  bool is_offboard_{false};

  // mavsdk
  mavsdk::Mavsdk *mavsdk_;
  std::shared_ptr<mavsdk::System> system_;
  mavsdk::Action *action_;
  mavsdk::Offboard *offboard_;
  mavsdk::Telemetry *telemetry_;
  mavsdk::MavlinkPassthrough *passthrough_;

  // methods
  bool initialize(const std::string &port);

  // services
  void getStatus(std::shared_ptr<raptor_interface::srv::QuadStatus::Request> request,
                  std::shared_ptr<raptor_interface::srv::QuadStatus::Response> response);

  void arm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void disarm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void startPosOffboard(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void stopPosOffboard(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void takeoff(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void land(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void goToPos(std::shared_ptr<raptor_interface::srv::GoToPos::Request> request,
                std::shared_ptr<raptor_interface::srv::GoToPos::Response> response);

  // service servers
  rclcpp::Service<raptor_interface::srv::QuadStatus>::SharedPtr service_quad_status_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_arm_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_disarm_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_start_offboard_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_stop_offboard_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_takeoff_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_land_;

  rclcpp::Service<raptor_interface::srv::GoToPos>::SharedPtr service_go_to_pos_;

  // publishers
  void positionPubCallback();

  // publisher servers
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_position_;
  rclcpp::TimerBase::SharedPtr timer_position_pub_;
};

// Helpers
std::string actionResultToString(mavsdk::Action::Result index);
std::string offboardResultToString(mavsdk::Offboard::Result index);

/**
 * Print error message when no port is provided as argument.
*/
void usage(const std::string &bin_name);

/**
 * Find PX4 flight controller.
 * 
 * @return Shared pointer to discovered system.
*/
std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk &mavsdk);
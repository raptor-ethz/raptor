#pragma once

// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"       // ros2 client library
#include "std_msgs/msg/string.hpp" // include std_msg type
#include "std_srvs/srv/trigger.hpp"

class Quad : public rclcpp::Node
{
public:
    Quad(mavsdk::Action *action, mavsdk::Offboard *offboard, mavsdk::Telemetry *telemetry);

private:
    void arm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void disarm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void takeoff(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void land(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void runPreflightCheck(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    std::string actionResultToString(mavsdk::Action::Result index);

    // mavsdk
    mavsdk::Action *action_;
    mavsdk::Offboard *offboard_;
    mavsdk::Telemetry *telemetry_;

    // ros
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_arm_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_disarm_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_preflight_check_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_takeoff_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_land_;
};
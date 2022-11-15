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

#include "geometry_msgs/msg/point.hpp"

class Quad : public rclcpp::Node
{
public:
    Quad(mavsdk::Action *action, mavsdk::Offboard *offboard, mavsdk::Telemetry *telemetry);

private:
    // Services
    void arm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void disarm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void start_pos_offboard(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void stop_pos_offboard(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void takeoff(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void land(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void runPreflightCheck(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Publisher
    void position_pub_callback();

    // Subscriber
    void position_ref_sub_callback(const geometry_msgs::msg::Point &msg);

    // Helpers
    std::string actionResultToString(mavsdk::Action::Result index);
    std::string offboardResultToString(mavsdk::Offboard::Result index);

    // mavsdk
    mavsdk::Action *action_;
    mavsdk::Offboard *offboard_;
    mavsdk::Telemetry *telemetry_;

    // ros
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_arm_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_disarm_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_pos_offboard_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_pos_offboard_;
    bool pos_offboard_active_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_preflight_check_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_takeoff_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_land_;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_ref_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};
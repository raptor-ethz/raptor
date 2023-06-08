#pragma once

#include "rclcpp/rclcpp.hpp"
#include "raptor_interface/srv/set_gripper.hpp"
#include "serialib.h"




class Gripper : public rclcpp::Node
{
public:
  using SetGripper = raptor_interface::srv::SetGripper;

  Gripper(const std::string &port);
  ~Gripper() {};

private:
  serialib serial_;
  // unsigned char cmd_[] = {90, 90}; // serial data packet
  std::array<unsigned char, 2> cmd_ = {90, 90};
  rclcpp::Service<SetGripper>::SharedPtr srv_set_gripper_;

  void setGripper(const std::shared_ptr<SetGripper::Request> request,
                  std::shared_ptr<SetGripper::Response> response);
};
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "raptor_interface/srv/rotate_gripper.hpp"
#include "serialib.h"




class Gripper : public rclcpp::Node
{
public:
  using RotateGripper = raptor_interface::srv::RotateGripper;

  Gripper(const std::string &port);
  ~Gripper() {};

private:
  serialib serial_;
  // unsigned char cmd_[] = {90, 90}; // serial data packet
  std::array<unsigned char, 2> cmd_ = {90, 90};
  rclcpp::Service<RotateGripper>::SharedPtr srv_set_gripper_;

  void rotateGripper(const std::shared_ptr<RotateGripper::Request> request,
                  std::shared_ptr<RotateGripper::Response> response);
};
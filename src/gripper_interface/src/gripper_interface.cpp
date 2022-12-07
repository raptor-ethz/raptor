#include <iostream>
#include <string>
// #include <chrono>

#include "raptor_interface/srv/set_servo.hpp"
#include "rclcpp/rclcpp.hpp"

void set_angle(
    std::shared_ptr<raptor_interface::srv::SetServo::Request> request,
    std::shared_ptr<raptor_interface::srv::SetServo::Response> response) {

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received arming request: [%d]",
  // request->data); if(request->data) response->success = true;
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Dending back response: [%d]",
  // response->success);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("gripper_interface_node");

  auto service = node->create_service<raptor_interface::srv::SetServo>(
      "frontGripper_Deg", &set_angle);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
#include <string>
#include <iostream>
// #include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

void arm_quad(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
              std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received arming request: [%d]", request->data);
  if(request->data) response->success = true;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Dending back response: [%d]", response->success);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arm_quad_server");

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service =
    node->create_service<std_srvs::srv::SetBool>("arm_quad", &arm_quad);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to arm quad.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
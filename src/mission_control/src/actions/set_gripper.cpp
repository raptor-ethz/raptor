#include "mission_control/mission_control.hpp"




bool MissionControl::setGripper(float left_angle_deg, float right_angle_deg) {
  // TODO check quad state

  // TODO check feasibility of given arguments

  // check if service is available TODO
  // if (!srv_arm_->wait_for_service(std::chrono::seconds(1))) {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arming service not found.");
  //   return false;
  // }

  auto request = std::make_shared<SetGripper::Request>(); // create request
  request->left_angle_deg = left_angle_deg;
  request->right_angle_deg = right_angle_deg;

  auto result = srv_set_gripper_->async_send_request(request); // send request


  // wait until service completed
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, std::chrono::milliseconds(500)) 
      == rclcpp::FutureReturnCode::SUCCESS) {
      
    auto response = result.get(); // get response from future

    if (!response->success) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to set griper.");
      return false; // service error
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper set to [%f, %f].",
        left_angle_deg, right_angle_deg);
    return true; // success

  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Set gripper service failed (ROS error).");
    return false; // service call unsuccessfull
  }
}
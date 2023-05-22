#include "mission_control/mission_control.hpp"




bool MissionControl::go_to_pos(const std::array<float, 3> &pos, const float yaw, const float timeout_s, const bool wait) {
  // TODO check quad state

  // check if service is available TODO
  if (!act_goToPos_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "GoToPos action not found.");
    return false;
  }

  // create first goal
  auto goal = GoToPos::Goal(); // create goal
  // set goal
  goal.x_m = pos[0];
  goal.y_m = pos[1];
  goal.z_m = pos[2];
  goal.yaw_deg = yaw;
  goal.timeout_s = timeout_s;
  goal.wait = wait;
  auto goal_handle_future = act_goToPos_->async_send_goal(goal); // request goal

  // wait until goal was processed
  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), goal_handle_future, std::chrono::seconds(1)) 
    != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "GoToPos action failed (ROS error).");
    return false;
  }

  // check if goal was accepted
  rclcpp_action::ClientGoalHandle<GoToPos>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "GoToPos goal rejected by server.");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Going to position [%f, %f, %f | %f].",
    pos[0], pos[1], pos[2], yaw);

  // waiting until action is completed
  auto result_future = act_goToPos_->async_get_result(goal_handle);
  // check return code
  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), result_future, std::chrono::seconds(20)) 
    != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "GoToPos action failed (ROS error).");
    return false;
  }
  
  // evaluate response
  auto result_wrapper = result_future.get();
  if (result_wrapper.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(this->get_logger(), "GoToPos action failed (%i).", result_wrapper.result->return_code);
    return false;
  }


  RCLCPP_INFO(this->get_logger(), "GoToPos successful.");
  return true;
}
#include "mission_control/mission_control.hpp"




bool MissionControl::acc_test(const std::array<float, 3> &acc, const std::array<float, 3> &threshold) {
  // TODO check quad state

  // check if service is available TODO
  if (!act_accTest->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "AccTest action not found.");
    return false;
  }

  // create first goal
  auto goal = AccTest::Goal(); // create goal
  // set goal
  goal.a_m_s2 = acc;
  goal.position_threshold_m = threshold;

  auto goal_handle_future = act_accTest->async_send_goal(goal); // request goal

  // wait until goal was processed
  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), goal_handle_future, std::chrono::seconds(1)) 
    != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "AccTest action failed (ROS error).");
    return false;
  }

  // check if goal was accepted
  rclcpp_action::ClientGoalHandle<AccTest>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "AccTest goal rejected by server.");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Testing acceleration [%f, %f, %f | %f, %f, %f].",
    acc[0], acc[1], acc[2], threshold[0], threshold[1], threshold[2]);

  // waiting until action is completed
  auto result_future = act_accTest->async_get_result(goal_handle);
  // check return code
  if (rclcpp::spin_until_future_complete(
    this->get_node_base_interface(), result_future, std::chrono::seconds(20)) 
    != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "AccTest action failed (ROS error).");
    return false;
  }
  
  // evaluate response
  auto result_wrapper = result_future.get();
  if (result_wrapper.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(this->get_logger(), "AccTest action failed (%i).", result_wrapper.result->return_code);
    return false;
  }


  RCLCPP_INFO(this->get_logger(), "AccTest successful.");
  return true;
}
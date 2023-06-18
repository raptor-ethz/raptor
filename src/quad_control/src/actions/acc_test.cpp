#include "quad_control/quad.hpp"




rclcpp_action::GoalResponse Quad::handleAccTestGoal(const rclcpp_action::GoalUUID & uuid,
                                                      std::shared_ptr<const AccTest::Goal> goal)
{
  (void) uuid;
  (void) goal;

  RCLCPP_INFO(this->get_logger(), "Received acceleration test goal request.");

  // check if request is valid TODO

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse Quad::handleAccTestCancel(const std::shared_ptr<AccTestGoalHandle> goal_handle)
{
  (void) goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received acceleration test cancel request.");
  // accept in any case
  return rclcpp_action::CancelResponse::ACCEPT;
}


void Quad::handleAccTestAccepted(const std::shared_ptr<AccTestGoalHandle> goal_handle)
{
  quad_state_->setState(State::POSITION);

  std::thread{std::bind(&Quad::executeAccTest, this, std::placeholders::_1), goal_handle}.detach();
}


void Quad::executeAccTest(const std::shared_ptr<AccTestGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  // auto feedback = std::make_shared<AccTest::Feedback>(); no feedback
  auto result = std::make_shared<AccTest::Result>();

  RCLCPP_INFO(this->get_logger(), "Executing acceleration test [%fms2 for %fm].",
    goal->a_m_s2[0], goal->position_threshold_m[0]);

  // get current position
  std::array<float, 3> initial_position = telemetry_->getPosition();

  int timeout_s = 10; // TODO
  float threshold = 0.3;

  int rate = 50;
  rclcpp::Rate loop_rate(rate);
  int max_iterations = timeout_s * rate;

  std::array<float, 3> acceleration_msg = {goal->a_m_s2[0], 0.0, 0.0};


  for (int i = 0; i < max_iterations; ++i) {
    if (!rclcpp::ok()) {return;} // check ROS shutdwon

    if (goal_handle->is_canceling()) { // check cancellation
      abortAccTest(goal_handle, 30, true);
      return;
    }

    // get current position
    std::array<float, 3> current_position = telemetry_->getPosition();

    // check safety margin in y, z and negative x direction
    if (current_position[0] - initial_position[0] < -threshold ||
        std::abs(current_position[1] - initial_position[1]) > threshold ||
        std::abs(current_position[2] - initial_position[2]) > threshold) {
      abortAccTest(goal_handle, 30, false);
      return;
    }

    // check position threshold
    if (current_position[0] - initial_position[0] > goal->position_threshold_m[0]) {
      mavsdk_wrapper_->sendPositionMessage(current_position); // stay at current position
      result->return_code = 0; // success
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Acceleration test execution successful.");
      return;
    }

    mavsdk_wrapper_->sendAccelerationMessage(acceleration_msg);
    loop_rate.sleep();
  }


  // timeout
  abortAccTest(goal_handle, 104, false);
}


void Quad::abortAccTest(const std::shared_ptr<AccTestGoalHandle> goal_handle, 
                    const int return_code, 
                    bool cancel)
{
  std::array<float, 3> position_msg = telemetry_->getPosition();
  position_msg[2] = 0.7; // safe height

  mavsdk_wrapper_->sendPositionMessage(position_msg); // stop

  RCLCPP_WARN(this->get_logger(), "Acceleration test stopped (%d). Maintaining current position.", 
    return_code);

  auto result = std::make_shared<AccTest::Result>();
  result->return_code = return_code;

  if (cancel) {
    goal_handle->canceled(result);
    return;
  }

  goal_handle->abort(result);
  return;
}
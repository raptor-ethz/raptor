#include "quad_control/quad.hpp"




rclcpp_action::GoalResponse Quad::handleHoverAccGoal(const rclcpp_action::GoalUUID & uuid,
                                                      std::shared_ptr<const HoverAcc::Goal> goal)
{
  (void) uuid;
  (void) goal;

  RCLCPP_INFO(this->get_logger(), "Received hover (acc) goal request.");

  // check if request is valid TODO

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse Quad::handleHoverAccCancel(const std::shared_ptr<HoverAccGoalHandle> goal_handle)
{
  (void) goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received hover (acc) cancel request.");
  // accept in any case
  return rclcpp_action::CancelResponse::ACCEPT;
}


void Quad::handleHoverAccAccepted(const std::shared_ptr<HoverAccGoalHandle> goal_handle)
{
  quad_state_->setState(State::POSITION);

  std::thread{std::bind(&Quad::executeHoverAcc, this, std::placeholders::_1), goal_handle}.detach();
}


void Quad::executeHoverAcc(const std::shared_ptr<HoverAccGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  // auto feedback = std::make_shared<HoverAcc::Feedback>(); no feedback
  auto result = std::make_shared<HoverAcc::Result>();

  RCLCPP_INFO(this->get_logger(), "Executing hover (acc) for %ds with threshold [%f, %f, %f].",
    goal->time_s,
    goal->pos_threshold_m[0], goal->pos_threshold_m[1], goal->pos_threshold_m[2]);

  // get current position
  std::array<float, 3> initial_position = telemetry_->getPosition();
  RCLCPP_INFO(this->get_logger(), "Initial position: [%f, %f, %f]",
    initial_position[0], initial_position[1], initial_position[2]);

  int rate = 50;
  rclcpp::Rate loop_rate(rate);
  int max_iterations = goal->time_s * rate;

  std::array<float, 3> acceleration_msg = {0.f, 0.f, 0.f};


  for (int i = 0; i < max_iterations; ++i) {
    if (!rclcpp::ok()) {return;} // check ROS shutdwon

    if (goal_handle->is_canceling()) { // check cancellation
      abortHoverAcc(goal_handle, 30, true);
      return;
    }

    // get current position
    std::array<float, 3> current_position = telemetry_->getPosition();

    // check safety margins
    float deviation_x = current_position[0] - initial_position[0];
    float deviation_y = current_position[1] - initial_position[1];
    float deviation_z = current_position[2] - initial_position[2];    
    if (std::abs(deviation_x) > goal->pos_threshold_m[0] ||
        std::abs(deviation_y) > goal->pos_threshold_m[1] ||
        std::abs(deviation_z) > goal->pos_threshold_m[2]) {
      RCLCPP_INFO(this->get_logger(), "Out of bounds [%f, %f, %f].",
        current_position[0], current_position[1], current_position[2]);
      abortHoverAcc(goal_handle, 100, false);
      return;
    }

    // P control
    float k_p = 2.f;
    acceleration_msg[0] = - k_p * deviation_x;
    acceleration_msg[1] = - k_p * deviation_y;
    acceleration_msg[2] = - k_p * deviation_z;

    mavsdk_wrapper_->sendAccelerationMessage(acceleration_msg);
    loop_rate.sleep();
  }


  // success
  result->return_code = 0;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Hover (acc) execution successful.");
}


void Quad::abortHoverAcc(const std::shared_ptr<HoverAccGoalHandle> goal_handle, 
                    const int return_code, 
                    bool cancel)
{
  std::array<float, 3> current_position = telemetry_->getPosition();

  mavsdk_wrapper_->sendPositionMessage(current_position); // hold current position

  RCLCPP_WARN(this->get_logger(), "Hover (acc) stopped (%d). Maintaining current position.", 
    return_code);

  auto result = std::make_shared<HoverAcc::Result>();
  result->return_code = return_code;

  if (cancel) {
    goal_handle->canceled(result);
    return;
  }

  goal_handle->abort(result);
  return;
}


bool Quad::doHoverAccStep(const std::array<float, 3> &initial_position,
                          const std::array<float, 3> &pos_threshold_m)
{
  // get current position
  std::array<float, 3> current_position = telemetry_->getPosition();

  // check safety margins
  std::array<float, 3> deviation = {current_position[0] - initial_position[0],
                                    current_position[1] - initial_position[1],
                                    current_position[2] - initial_position[2]}
  if (std::abs(deviation[0]) > pos_threshold_m[0] ||
      std::abs(deviation[1]) > pos_threshold_m[1] ||
      std::abs(deviation[2]) > pos_threshold_m[2]) {
    RCLCPP_INFO(this->get_logger(), "Out of bounds [%f, %f, %f].",
      current_position[0], current_position[1], current_position[2]);
    return false;
  }

  // P control
  float k_p = 2.f;
  std::array<float, 3> acceleration_msg = {- k_p * deviation[0],
                                           - k_p * deviation[1],
                                           - k_p * deviation[2]};

  mavsdk_wrapper_->sendAccelerationMessage(acceleration_msg);
  return true;
}
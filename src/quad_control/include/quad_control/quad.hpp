#pragma once

#include <string>
#include <iostream>
#include <chrono>

// ros default
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// custom
#include "quad_control/mavsdk_wrapper.hpp"
#include "quad_control/quad_state.hpp"
#include "quad_control/telemetry.hpp"

// interfaces
#include "raptor_interface/srv/trigger.hpp"
#include "raptor_interface/msg/pose.hpp"
#include "raptor_interface/msg/velocity.hpp"
#include "raptor_interface/action/takeoff.hpp"
#include "raptor_interface/action/go_to_pos.hpp"

// mavsdk
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>



class Quad : public rclcpp::Node
{
public:
  // interface type aliases
  using Pose = raptor_interface::msg::Pose;
  using Velocity = raptor_interface::msg::Velocity;
  using Trigger = raptor_interface::srv::Trigger;
  using Takeoff = raptor_interface::action::Takeoff;
  using TakeoffGoalHandle = rclcpp_action::ServerGoalHandle<Takeoff>;
  using GoToPos = raptor_interface::action::GoToPos;
  using GoToPosGoalHandle = rclcpp_action::ServerGoalHandle<GoToPos>;

  Quad(const std::string &port);
  ~Quad() {}; // TODO should we explicitly reset the shared pointers here?


private:
  // modules
  std::shared_ptr<QuadState> quad_state_;
  std::shared_ptr<MavsdkWrapper> mavsdk_wrapper_;
  std::shared_ptr<Telemetry> telemetry_;

  // interface servers
  rclcpp::Service<Trigger>::SharedPtr srv_arm_;
  rclcpp::Service<Trigger>::SharedPtr srv_land_;
  rclcpp_action::Server<Takeoff>::SharedPtr act_takeoff_;
  rclcpp_action::Server<GoToPos>::SharedPtr act_goToPos_;

  // interface clients
  rclcpp::Subscription<Pose>::SharedPtr sub_pose_;
  rclcpp::Subscription<Velocity>::SharedPtr sub_vel_;

  // subscriptions
  void pose_callback(const Pose::SharedPtr msg);
  void vel_callback(const Velocity::SharedPtr msg);

  // services
  void arm(const std::shared_ptr<Trigger::Request> request,
            std::shared_ptr<Trigger::Response> response);
  void land(const std::shared_ptr<Trigger::Request> request,
            std::shared_ptr<Trigger::Response> response);

  // 
  // ACTIONS
  // 

  // takeoff
  rclcpp_action::GoalResponse handleTakeoffGoal(const rclcpp_action::GoalUUID & uuid,
                                                std::shared_ptr<const Takeoff::Goal> goal);
  rclcpp_action::CancelResponse handleTakeoffCancel(
    const std::shared_ptr<TakeoffGoalHandle> goal_handle);
  void handleTakeoffAccepted(const std::shared_ptr<TakeoffGoalHandle> goal_handle);
  void executeTakeoff(const std::shared_ptr<TakeoffGoalHandle> goal_handle);
  void abortTakeoff(const std::shared_ptr<TakeoffGoalHandle> goal_handle, 
                    const int return_code, 
                    bool cancel = false);
  // go to pos
  rclcpp_action::GoalResponse handleGoToPosGoal(const rclcpp_action::GoalUUID & uuid,
                                                std::shared_ptr<const GoToPos::Goal> goal);
  rclcpp_action::CancelResponse handleGoToPosCancel(
    const std::shared_ptr<GoToPosGoalHandle> goal_handle);
  void handleGoToPosAccepted(const std::shared_ptr<GoToPosGoalHandle> goal_handle);
  void executeGoToPos(const std::shared_ptr<GoToPosGoalHandle> goal_handle);
  void abortGoToPos(const std::shared_ptr<GoToPosGoalHandle> goal_handle, 
                    const int return_code, 
                    bool cancel = false);
};

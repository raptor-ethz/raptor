#pragma once

#include <string>

// ros default
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// custom
#include "mission_control/telemetry.hpp"

// interfaces
#include "raptor_interface/msg/pose.hpp"
#include "raptor_interface/srv/trigger.hpp"
#include "raptor_interface/action/takeoff.hpp"
#include "raptor_interface/action/go_to_pos.hpp"




class MissionControl : public rclcpp::Node
{
public:
  using Pose = raptor_interface::msg::Pose;
  using Trigger = raptor_interface::srv::Trigger;
  using Takeoff = raptor_interface::action::Takeoff;
  using TakeoffGoalHandle = rclcpp_action::ClientGoalHandle<Takeoff>;
  using GoToPos = raptor_interface::action::GoToPos;
  using GoToPosGoalHandle = rclcpp_action::ClientGoalHandle<GoToPos>;

  MissionControl();
  ~MissionControl() {};

  // API
  bool arm();
  bool takeoff(const float altitude);
  bool land();
  bool go_to_pos(const std::array<float, 3> &pos, const float yaw, const float timeout_s, const bool wait = false);
  bool go_to_object(const std::array<float, 4> &offset, const float yaw, const float timeout_s, const bool wait = false);

  // helper functions
  void shutdown();

private:
  // TODO add quad state

  std::shared_ptr<Telemetry> object_telemetry_;

  // interface clients
  rclcpp::Subscription<Pose>::SharedPtr sub_object_pose_;
  rclcpp::Client<Trigger>::SharedPtr srv_arm_;
  rclcpp::Client<Trigger>::SharedPtr srv_land_;
  rclcpp_action::Client<Takeoff>::SharedPtr act_takeoff_;
  rclcpp_action::Client<GoToPos>::SharedPtr act_goToPos_;

  // subscriptions
  void objectPoseCallback(const Pose::SharedPtr msg);
};
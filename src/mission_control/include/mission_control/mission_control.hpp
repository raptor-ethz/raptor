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
#include "raptor_interface/srv/set_gripper.hpp"
#include "raptor_interface/action/takeoff.hpp"
#include "raptor_interface/action/go_to_pos.hpp"
#include "raptor_interface/action/acc_test.hpp"




class MissionControl : public rclcpp::Node
{
public:
  using Pose = raptor_interface::msg::Pose;
  using Trigger = raptor_interface::srv::Trigger;
  using SetGripper = raptor_interface::srv::SetGripper;
  using Takeoff = raptor_interface::action::Takeoff;
  using TakeoffGoalHandle = rclcpp_action::ClientGoalHandle<Takeoff>;
  using GoToPos = raptor_interface::action::GoToPos;
  using GoToPosGoalHandle = rclcpp_action::ClientGoalHandle<GoToPos>;
  using AccTest = raptor_interface::action::AccTest;
  using AccTestGoalHandle = rclcpp_action::ServerGoalHandle<AccTest>;

  MissionControl();
  ~MissionControl() {};

  // API
  bool arm();
  bool setGripper(float left_deg, float right_deg);
  bool takeoff(const float altitude);
  bool land();
  bool go_to_pos(const std::array<float, 3> &pos, const float yaw, const float timeout_s, const bool wait = false);
  bool go_to_object(const std::array<float, 3> &offset, const float yaw, const float timeout_s, const bool wait = false);
  bool acc_test(const std::array<float, 3> &acc, const std::array<float, 3> &threshold);

  // helper functions
  void shutdown();

private:
  // TODO add quad state

  std::shared_ptr<Telemetry> object_telemetry_;

  // interface clients
  rclcpp::Subscription<Pose>::SharedPtr sub_object_pose_;
  rclcpp::Client<Trigger>::SharedPtr srv_arm_;
  rclcpp::Client<Trigger>::SharedPtr srv_land_;
  rclcpp::Client<SetGripper>::SharedPtr srv_set_gripper_;
  rclcpp_action::Client<Takeoff>::SharedPtr act_takeoff_;
  rclcpp_action::Client<GoToPos>::SharedPtr act_goToPos_;
  rclcpp_action::Client<AccTest>::SharedPtr act_accTest_;

  // subscriptions
  void objectPoseCallback(const Pose::SharedPtr msg);
};
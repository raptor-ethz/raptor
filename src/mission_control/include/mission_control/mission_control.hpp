#pragma once

#include <string>

// ros default
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// interfaces
#include "raptor_interface/srv/trigger.hpp"
#include "raptor_interface/action/takeoff.hpp"
#include "raptor_interface/action/go_to_pos.hpp"




class MissionControl : public rclcpp::Node
{
public:
  using Takeoff = raptor_interface::action::Takeoff;
  using TakeoffGoalHandle = rclcpp_action::ClientGoalHandle<Takeoff>;
  using GoToPos = raptor_interface::action::GoToPos;
  using GoToPosGoalHandle = rclcpp_action::ClientGoalHandle<GoToPos>;

  MissionControl();
  ~MissionControl() {};


  // API
  bool arm();

  bool takeoff(const float altitude);

  bool go_to_pos(const std::array<float, 3> &pos, const float yaw = 0);


private:
  // TODO add quad state

  rclcpp_action::Client<Takeoff>::SharedPtr act_takeoff_;
  rclcpp_action::Client<GoToPos>::SharedPtr act_goToPos_;

};
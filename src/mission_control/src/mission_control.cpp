#include "mission_control/mission_control.hpp"


const std::string ACTION_TAKEOFF = "takeoff";
const std::string ACTION_GOTOPOS = "goToPos";




MissionControl::MissionControl() : Node("mission_control")
{
  RCLCPP_INFO(this->get_logger(), "Initializing...");


  // initialize ros interface clients
  act_takeoff_ = rclcpp_action::create_client<Takeoff>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    ACTION_TAKEOFF);

  act_goToPos_ = rclcpp_action::create_client<GoToPos>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    ACTION_GOTOPOS);



  // check if servers are available
  if(!act_takeoff_->wait_for_action_server(std::chrono::seconds(3)))
  {
    RCLCPP_WARN(this->get_logger(), "Takeoff action server not available");
  }

  if(!act_goToPos_->wait_for_action_server(std::chrono::seconds(3)))
  {
    RCLCPP_WARN(this->get_logger(), "GoToPos action server not available");
  }

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}
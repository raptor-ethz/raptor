#include "mission_control/mission_control.hpp"


const std::string ACT_LABEL_TAKEOFF = "takeoff";
const std::string ACT_LABEL_GOTOPOS = "goToPos";




MissionControl::MissionControl() : Node("mission_control")
{
  RCLCPP_INFO(this->get_logger(), "Initializing...");


  // initialize ros interface clients
  srv_arm_ = this->create_client<Trigger>("arm");
  srv_land_ = this->create_client<Trigger>("land");
  act_takeoff_ = rclcpp_action::create_client<Takeoff>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    ACT_LABEL_TAKEOFF);

  act_goToPos_ = rclcpp_action::create_client<GoToPos>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    ACT_LABEL_GOTOPOS);



  // check if servers are available
  // TODO: Parameters to see if error, warning or nothing should be returned if specific server is not available
  if(!srv_arm_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(this->get_logger(), "Arming service not available");
    rclcpp::shutdown();
    return;
  }
  if(!srv_land_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(this->get_logger(), "Landing service not available");
    rclcpp::shutdown();
    return;
  }
  if(!act_takeoff_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(this->get_logger(), "Takeoff action server not available"); 
    rclcpp::shutdown();
    return;
  }
  if(!act_goToPos_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(this->get_logger(), "GoToPos action server not available");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}
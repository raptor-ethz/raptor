#include "mission_control/mission_control.hpp"


const std::string ACT_LABEL_TAKEOFF = "takeoff";
const std::string ACT_LABEL_GOTOPOS = "go_to_pos";
const std::string ACT_LABE_ACCTEST = "accTest";




MissionControl::MissionControl() : Node("mission_control")
{
  using namespace std::placeholders;

  RCLCPP_INFO(this->get_logger(), "Initializing...");

  object_telemetry_ = std::make_shared<Telemetry>();

  // subscriptions
  sub_object_pose_ = this->create_subscription<Pose>(
    "object_pose_nwu", 10, std::bind(&MissionControl::objectPoseCallback, this, _1));

  // service clients
  srv_arm_ = this->create_client<Trigger>("arm");
  srv_land_ = this->create_client<Trigger>("land");
  srv_set_gripper_ = this->create_client<SetGripper>("set_gripper");

  // action clients
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

  act_accTest_ = rclcpp_action::create_client<AccTest>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    ACT_LABE_ACCTEST);



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


void MissionControl::shutdown() {
  RCLCPP_ERROR(this->get_logger(), "Mission interrupted. Shutting down");
  rclcpp::shutdown();
  exit(0);
}


void MissionControl::objectPoseCallback(const Pose::SharedPtr msg) {
  object_telemetry_->setPosition({msg->x_m, msg->y_m, msg->z_m});
  // TODO attitude information
}
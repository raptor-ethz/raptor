#include "quad.hpp"

// TODO read these values from a yaml file
const int position_pub_interval = 50;

// Constructor
Quad::Quad(mavsdk::Action *action, mavsdk::Offboard *offboard, mavsdk::Telemetry *telemetry) 
  : Node("quad_interface")
{
  // init mavsdk variables
  action_ = action;
  offboard_ = offboard;
  telemetry_ = telemetry;

  // initialize member variables
  pos_offboard_active_ = false;

  // create services
  service_arm_ = this->create_service<std_srvs::srv::Trigger>(
    "arm", std::bind( &Quad::arm, 
                      this, 
                      std::placeholders::_1, 
                      std::placeholders::_2));
  service_preflight_check_ = this->create_service<std_srvs::srv::Trigger>(
    "preflight_check", std::bind( &Quad::runPreflightCheck, 
                                  this, 
                                  std::placeholders::_1, 
                                  std::placeholders::_2));
  service_disarm_ = this->create_service<std_srvs::srv::Trigger>(
    "disarm", std::bind(&Quad::disarm, 
                        this, 
                        std::placeholders::_1, 
                        std::placeholders::_2));
  service_takeoff_ = this->create_service<std_srvs::srv::Trigger>(
    "takeoff", std::bind( &Quad::takeoff, 
                          this, std::placeholders::_1, 
                          std::placeholders::_2));
  service_land_ = this->create_service<std_srvs::srv::Trigger>(
    "land", std::bind(&Quad::land, 
                      this, 
                      std::placeholders::_1, 
                      std::placeholders::_2));
  start_pos_offboard_ = this->create_service<std_srvs::srv::Trigger>(
    "start_pos_offboard", std::bind(&Quad::startPosOffboard, 
                                    this, 
                                    std::placeholders::_1, 
                                    std::placeholders::_2));
  stop_pos_offboard_ = this->create_service<std_srvs::srv::Trigger>(
    "stop_pos_offboard", std::bind( &Quad::stopPosOffboard, 
                                    this, 
                                    std::placeholders::_1, 
                                    std::placeholders::_2));

  // create publishers
  pub_position_ = this->create_publisher<geometry_msgs::msg::Point>("position", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(position_pub_interval), std::bind(&Quad::positionPubCallback, this));


  //   this->action_server_ = rclcpp_action::create_server<Fibonacci>(
  //     this,
  //     "fibonacci",
  //     std::bind(&FibonacciActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
  //     std::bind(&FibonacciActionServer::handle_cancel, this, std::placeholders::_1),
  //     std::bind(&FibonacciActionServer::handle_accepted, this, std::placeholders::_1));
  // }


}

// Services
void Quad::arm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received arming request");
  const mavsdk::Action::Result arm_result = action_->arm();
  response->message = this->actionResultToString(arm_result);
  if (arm_result == mavsdk::Action::Result::Success)
  {
    // wait for drone to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    response->success = true;
  }
  else
  {
    response->success = false;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Arming result: [%s]", response->message.c_str());
}

void Quad::disarm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received disarming request");
  const mavsdk::Action::Result disarm_result = action_->disarm();
  response->message = this->actionResultToString(disarm_result);
  if (disarm_result == mavsdk::Action::Result::Success)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disarming result: [%s]", response->message.c_str());
}

void Quad::startPosOffboard(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received position offboard starting request");
  // send offboard message once before starting
  mavsdk::Offboard::PositionNedYaw pos_msg{};
  pos_msg.north_m = telemetry_->position_velocity_ned().position.north_m;
  pos_msg.east_m = telemetry_->position_velocity_ned().position.east_m;
  pos_msg.down_m = telemetry_->position_velocity_ned().position.down_m;
  pos_msg.yaw_deg = 0.0; // TODO
  // starting offboard
  const mavsdk::Offboard::Result offboard_result = offboard_->start();
  response->message = this->offboardResultToString(offboard_result);
  if (offboard_result == mavsdk::Offboard::Result::Success)
  {
    pos_offboard_active_ = true;
    response->success = true;
  }
  else
  {
    response->success = false;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
    "Position offboard start result: [%s]", response->message.c_str());
}

void Quad::stopPosOffboard(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received position offboard stopping request");
  const mavsdk::Offboard::Result offboard_result = offboard_->stop();
  response->message = this->offboardResultToString(offboard_result);
  if (offboard_result == mavsdk::Offboard::Result::Success)
  {
    pos_offboard_active_ = false;
    response->success = true;
  }
  else
  {
    response->success = false;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Position offboard stop result: [%s]", response->message.c_str());
}

void Quad::takeoff(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received takeoff request");
  const mavsdk::Action::Result takeoff_result = action_->takeoff();
  response->message = this->actionResultToString(takeoff_result);
  if (takeoff_result == mavsdk::Action::Result::Success)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Takeoff result: [%s]", response->message.c_str());
}

void Quad::land(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received land request");
  const mavsdk::Action::Result land_result = action_->land();
  response->message = this->actionResultToString(land_result);
  if (land_result == mavsdk::Action::Result::Success)
  {
    response->success = true;
  }
  else
  {
    response->success = false;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Land result: [%s]", response->message.c_str());
}

void Quad::runPreflightCheck(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received preflight check request");
  // if(request->data) response->success = true;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back preflicht response: [%d]", response->success);
}

// Publishers
void Quad::positionPubCallback()
{
  auto message = geometry_msgs::msg::Point();
  message.x = telemetry_->position_velocity_ned().position.north_m;
  message.y = telemetry_->position_velocity_ned().position.east_m;
  message.z = -telemetry_->position_velocity_ned().position.down_m;
  // RCLCPP_INFO(this->get_logger(), "Publishing: [%f,%f,%f]", message.x, message.y, message.z);
  pub_position_->publish(message);
}

// Actions
void Quad::GoToPos(){
  
}

// Helpers
std::string Quad::actionResultToString(mavsdk::Action::Result index)
{
  std::string val[] =
      {"Unknown", "Success", "NoSystem", "ConnectionError", "Busy", "CommandDenied", "CommandDeniedLandedStateUnknown", "CommandDeniedNotLanded", "Timeout", "VtolTransitionSupportUnknown", "NoVtolTransitionSupport", "ParameterError", "Unsupported"};
  return val[int(index)];
}

std::string Quad::offboardResultToString(mavsdk::Offboard::Result index)
{
  std::string val[] =
      {"Unknown", "Success", "NoSystem", "ConnectionError", "Busy", "CommandDenied", "Timeout", "NoSetpointSet"};
  return val[int(index)];
}

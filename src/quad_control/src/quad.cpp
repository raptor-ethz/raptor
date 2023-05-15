#include "quad_control/quad.hpp"

// TODO read these values from a yaml file
const int position_pub_interval = 50;
const string TOPIC_POSE = "px4_pose_nwu";
const string TOPIC_VEL = "px4_vel_nwu";

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// Construction

// Constructor
Quad::Quad(const std::string &port) : Node("quad_control") {

  using namespace std::placeholders;

  RCLCPP_INFO(this->get_logger(), "Initializing...");


  // initialize modules
  quad_state_ = std::make_shared<QuadState>();
  mavsdk_wrapper_ = std::make_shared<MavsdkWrapper>();
  telemetry_ = std::make_shared<Telemetry>();

  if (mavsdk_wrapper_->initialize(port)) {
    RCLCPP_ERROR(this->get_logger(), "MavsdkWrapper initialization failed.");
    rclcpp::shutdown();
    return;
  }


  // subscriptions
  sub_pose_ =  this->create_subscription<raptor_interface::msg::Pose>(
                TOPIC_POSE, 10, std::bind(&Quad::pose_callback, this, _1));

  sub_vel_ = this->create_subscription<raptor_interface::msg::Velocity>(
              TOPIC_VEL, 10, std::bind(&Quad::vel_callback, this, _1));


  // service servers
  srv_arm_ = this->create_service<Trigger>(
              "arm", std::bind(&Quad::arm, this, _1, _2));
  srv_land_ = this->create_service<Trigger>(
              "land", std::bind(&Quad::land, this, _1, _2));


  // action servers
  act_takeoff_ = rclcpp_action::create_server<Takeoff>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "takeoff",
    std::bind(&Quad::handleTakeoffGoal, this, _1, _2),
    std::bind(&Quad::handleTakeoffCancel, this, _1),
    std::bind(&Quad::handleTakeoffAccepted, this, _1));

  act_goToPos_ = rclcpp_action::create_server<GoToPos>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "goToPos",
    std::bind(&Quad::handleGoToPosGoal, this, _1, _2),
    std::bind(&Quad::handleGoToPosCancel, this, _1),
    std::bind(&Quad::handleGoToPosAccepted, this, _1));



  quad_state_->setState(State::INITIALIZED);

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////// Subscriptions

void Quad::pose_callback(const Pose & msg)
{
  telemetry_->setPosition({msg.x_m, msg.y_m, msg.z_m});
  telemetry_->setAttitude({msg.roll_deg, msg.pitch_deg, msg.yaw_deg});
}

void Quad::vel_callback(const Velocity & msg)
{
  telemetry_->setVelocity({msg.x_m_s, msg.y_m_s, msg.z_m_s});
}




////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////// Services

void Quad::arm(std::shared_ptr<Trigger::Request> request,
               std::shared_ptr<Trigger::Response> response) {
  (void)request; // suppress unused variable warning

  RCLCPP_INFO(this->get_logger(), "Received arm request.");

  // check if proper state
  if (!quad_state_->isValidTransition(State::ARMED)) {
    response->result = 201; // request not feasible
    return;
  }

  const int mavsdk_result = mavsdk_wrapper_->sendArmRequest();

  if (mavsdk_result == 1) {
    response->result = 0; // success
    quad_state_->setState(State::ARMED);
  } else {
    response->result = mavsdk_result + 300; // mavsdk error
  }
  
  // debug
  RCLCPP_INFO(this->get_logger(), "Arm result: [%d]", response->result);
}


void Quad::land(std::shared_ptr<Trigger::Request> request,
               std::shared_ptr<Trigger::Response> response) {
  (void)request; // suppress unused variable warning

  RCLCPP_INFO(this->get_logger(), "Received land request.");

  // check if proper state TODO

  const int mavsdk_result = mavsdk_wrapper_->sendLandRequest();

  if (mavsdk_result == 1) {
    response->result = 0; // success
    quad_state_->setState(State::INITIALIZED); // TODO
  } else {
    response->result = mavsdk_result + 300; // mavsdk error
  }
  
  // debug
  RCLCPP_INFO(this->get_logger(), "Land result: [%d]", response->result);
}


// void Quad::getStatus(
//     std::shared_ptr<raptor_interface::srv::QuadStatus::Request> request,
//     std::shared_ptr<raptor_interface::srv::QuadStatus::Response> response) {
//   (void)request;
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received status request.");
//   response->battery = telemetry_->battery().remaining_percent * 100.0;
//   response->local_pos_ok = telemetry_->health().is_local_position_ok;
//   response->armable = telemetry_->health().is_armable;
//   // debug
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back status: [%f, %i, %i]",
//               response->battery, response->local_pos_ok, response->armable);
// }


// void Quad::disarm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
//                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
//   (void)request;
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received disarming request.");
//   const mavsdk::Action::Result disarm_result = action_->disarm();
//   response->message = actionResultToString(disarm_result);
//   if (disarm_result == mavsdk::Action::Result::Success) {
//     response->success = true;
//   } else {
//     response->success = false;
//   }
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disarming result: [%s]",
//               response->message.c_str());
// }

// void Quad::startPosOffboard(
//     std::shared_ptr<std_srvs::srv::Trigger::Request> request,
//     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
//   (void)request;
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
//               "Received position offboard starting request.");
//   // send offboard message once before starting TODO
//   mavsdk::Offboard::PositionNedYaw pos_msg{};
//   pos_msg.north_m = telemetry_->position_velocity_ned().position.north_m;
//   pos_msg.east_m = telemetry_->position_velocity_ned().position.east_m;
//   pos_msg.down_m = telemetry_->position_velocity_ned().position.down_m;
//   pos_msg.yaw_deg = 0.0; // TODO
//   // starting offboard
//   offboard_->set_position_ned(pos_msg);
//   const mavsdk::Offboard::Result offboard_result = offboard_->start();
//   response->message = offboardResultToString(offboard_result);
//   offboard_->set_position_ned(pos_msg);
//   if (offboard_result == mavsdk::Offboard::Result::Success) {
//     is_offboard_ = true;
//     response->success = true;
//   } else {
//     response->success = false;
//   }
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
//               "Position offboard start result: [%s]",
//               response->message.c_str());
// }

// void Quad::stopPosOffboard(
//     std::shared_ptr<std_srvs::srv::Trigger::Request> request,
//     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
//   (void)request;
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
//               "Received position offboard stopping request.");
//   const mavsdk::Offboard::Result offboard_result = offboard_->stop();
//   response->message = offboardResultToString(offboard_result);
//   if (offboard_result == mavsdk::Offboard::Result::Success) {
//     is_offboard_ = false;
//     response->success = true;
//   } else {
//     response->success = false;
//   }
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
//               "Position offboard stop result: [%s]", response->message.c_str());
// }

// void Quad::takeoff(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
//                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
//   (void)request;
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received takeoff request.");
//   const mavsdk::Action::Result takeoff_result = action_->takeoff();
//   response->message = actionResultToString(takeoff_result);
//   if (takeoff_result == mavsdk::Action::Result::Success) {
//     response->success = true;
//   } else {
//     response->success = false;
//   }
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Takeoff result: [%s]",
//               response->message.c_str());
// }

// void Quad::land(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
//                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
//   (void)request;
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received land request.");
//   const mavsdk::Action::Result land_result = action_->land();
//   response->message = actionResultToString(land_result);
//   if (land_result == mavsdk::Action::Result::Success) {
//     response->success = true;
//   } else {
//     response->success = false;
//   }
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Land result: [%s]",
//               response->message.c_str());
// }

// void Quad::goToPos(
//     std::shared_ptr<raptor_interface::srv::GoToPos::Request> request,
//     std::shared_ptr<raptor_interface::srv::GoToPos::Response> response) {
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
//               "Received GoToPos request: [%f, %f, %f, %f]", request->x_ref,
//               request->y_ref, request->z_ref, request->yaw_ref);
//   // create message
//   mavsdk::Offboard::PositionNedYaw pos_msg{};
//   pos_msg.north_m = request->x_ref;
//   pos_msg.east_m = -request->y_ref;
//   pos_msg.down_m = -request->z_ref;
//   pos_msg.yaw_deg = request->yaw_ref;
//   // send message to quad
//   offboard_->set_position_ned(pos_msg);

//   response->success = true;
// }

// ////////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////// Publishers

// void Quad::positionPubCallback() {
//   auto message = geometry_msgs::msg::Point();
//   message.x = telemetry_->position_velocity_ned().position.north_m;
//   message.y = -telemetry_->position_velocity_ned().position.east_m;
//   message.z = -telemetry_->position_velocity_ned().position.down_m;
//   // debug
//   // RCLCPP_INFO(this->get_logger(), "Publishing Position: [%f,%f,%f]",
//   // message.x, message.y, message.z);
//   pub_position_->publish(message);
// }

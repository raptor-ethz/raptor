#include "quad_control/quad.hpp"

// TODO read these values from a yaml file
const int position_pub_interval = 50;

const std::string MAVSDK_OFFBOARD_RESULTS[] = { "Unknown",
                                                "Success",
                                                "NoSystem",
                                                "ConnectionError",
                                                "Busy",
                                                "CommandDenied",
                                                "Timeout",
                                                "NoSetpointSet"};

const std::string MAVSDK_ACTION_RESULTS[] = { "Unknown",
                                              "Success",
                                              "NoSystem",
                                              "ConnectionError",
                                              "Busy",
                                              "CommandDenied",
                                              "CommandDeniedLandedStateUnknown",
                                              "CommandDeniedNotLanded",
                                              "Timeout",
                                              "VtolTransitionSupportUnknown",
                                              "NoVtolTransitionSupport",
                                              "ParameterError",
                                              "Unsupported"};

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// Construction

// Constructor
Quad::Quad(const std::string &port) : Node("quad_control") {

  using namespace std::placeholders;

  RCLCPP_INFO(this->get_logger(), "Initializing...");


  mavsdk_wrapper_ = std::make_shared<MavsdkWrapper>();

  if (mavsdk_wrapper_->initialize(port)) {
    RCLCPP_ERROR(this->get_logger(), "MavsdkWrapper initialization failed.");
    rclcpp::shutdown();
    return;
  }




  // service servers
  srv_arm_ = this->create_service<raptor_interface::srv::Trigger>(
              "arm", std::bind(&Quad::arm, this, _1, _2));

  // TODO create action servers here





  quad_state_->setState(State::INITIALIZED);

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");

  // service_quad_status_ =
  //     this->create_service<raptor_interface::srv::QuadStatus>(
  //         "quad_status",
  //         std::bind(&Quad::getStatus, this, std::placeholders::_1,
  //                   std::placeholders::_2));
  // service_disarm_ = this->create_service<std_srvs::srv::Trigger>(
  //     "disarm", std::bind(&Quad::disarm, this, std::placeholders::_1,
  //                         std::placeholders::_2));
  // service_takeoff_ = this->create_service<std_srvs::srv::Trigger>(
  //     "takeoff", std::bind(&Quad::takeoff, this, std::placeholders::_1,
  //                          std::placeholders::_2));
  // service_land_ = this->create_service<std_srvs::srv::Trigger>(
  //     "land", std::bind(&Quad::land, this, std::placeholders::_1,
  //                       std::placeholders::_2));
  // service_start_offboard_ = this->create_service<std_srvs::srv::Trigger>(
  //     "start_offboard",
  //     std::bind(&Quad::startPosOffboard, this, std::placeholders::_1,
  //               std::placeholders::_2));
  // service_stop_offboard_ = this->create_service<std_srvs::srv::Trigger>(
  //     "stop_offboard", std::bind(&Quad::stopPosOffboard, this,
  //                                std::placeholders::_1, std::placeholders::_2));
  // service_go_to_pos_ = this->create_service<raptor_interface::srv::GoToPos>(
  //     "go_to_pos", std::bind(&Quad::goToPos, this, std::placeholders::_1,
  //                            std::placeholders::_2));

  // create publishers
  // pub_position_ =
  //     this->create_publisher<geometry_msgs::msg::Point>("position", 10);
  // timer_position_pub_ =
  //     this->create_wall_timer(std::chrono::milliseconds(position_pub_interval),
  //                             std::bind(&Quad::positionPubCallback, this));
}





////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////// Services

void Quad::arm(std::shared_ptr<raptor_interface::srv::Trigger::Request> request,
               std::shared_ptr<raptor_interface::srv::Trigger::Response> response) {
  (void)request; // suppress unused variable warning

  RCLCPP_INFO(this->get_logger(), "Received arm request.");

  // check if proper state TODO
  if (!quad_state_->isValidTransition(State::ARMED)) {
    response->result = 201; // request not feasible
    return;
  }

  const int mavsdk_result = mavsdk_wrapper_->sendArmRequest();

  if (mavsdk_result == 1) {
    response->result = 0; // success
  } else {
    response->result = mavsdk_result + 300; // mavsdk error
  }
  
  // debug
  RCLCPP_INFO(this->get_logger(), "Arm result: [%d]", response->result);
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




// ////////////////////////////////////////////////////////////////////////////////
// //////////////////////////////////////////////////////////////////////// Helpers

std::string actionResultToString(mavsdk::Action::Result index) {
  return MAVSDK_ACTION_RESULTS[int(index)];
}


std::string offboardResultToString(mavsdk::Offboard::Result index) {
  return MAVSDK_OFFBOARD_RESULTS[int(index)];
}


void usage(const std::string &bin_name) {
  std::cerr
      << "Usage : " << bin_name << " <connection_url>\n"
      << "Connection URL format should be :\n"
      << " For TCP : tcp://[server_host][:server_port]\n"
      << " For UDP : udp://[bind_host][:bind_port]\n"
      << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
      << "For example, to connect to the simulator use URL: udp://:14540\n";
}

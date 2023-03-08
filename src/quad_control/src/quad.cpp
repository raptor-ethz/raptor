#include "quad_control/quad_control.hpp"

// TODO read these values from a yaml file
const int position_pub_interval = 50;

// Constructor
Quad::Quad() : Node("quad_interface") {
  // initialize member variables
  initialized_ = false;
  is_airborne_ = false;
  is_offboard_ = false;

  // create services
  service_quad_status_ =
      this->create_service<raptor_interface::srv::QuadStatus>(
          "quad_status",
          std::bind(&Quad::getStatus, this, std::placeholders::_1,
                    std::placeholders::_2));
  service_arm_ = this->create_service<std_srvs::srv::Trigger>(
      "arm", std::bind(&Quad::arm, this, std::placeholders::_1,
                       std::placeholders::_2));
  service_disarm_ = this->create_service<std_srvs::srv::Trigger>(
      "disarm", std::bind(&Quad::disarm, this, std::placeholders::_1,
                          std::placeholders::_2));
  service_takeoff_ = this->create_service<std_srvs::srv::Trigger>(
      "takeoff", std::bind(&Quad::takeoff, this, std::placeholders::_1,
                           std::placeholders::_2));
  service_land_ = this->create_service<std_srvs::srv::Trigger>(
      "land", std::bind(&Quad::land, this, std::placeholders::_1,
                        std::placeholders::_2));
  service_start_offboard_ = this->create_service<std_srvs::srv::Trigger>(
      "start_offboard",
      std::bind(&Quad::startPosOffboard, this, std::placeholders::_1,
                std::placeholders::_2));
  service_stop_offboard_ = this->create_service<std_srvs::srv::Trigger>(
      "stop_offboard", std::bind(&Quad::stopPosOffboard, this,
                                 std::placeholders::_1, std::placeholders::_2));
  service_go_to_pos_ = this->create_service<raptor_interface::srv::GoToPos>(
      "go_to_pos", std::bind(&Quad::goToPos, this, std::placeholders::_1,
                             std::placeholders::_2));

  // create publishers
  pub_position_ =
      this->create_publisher<geometry_msgs::msg::Point>("position", 10);
  timer_position_pub_ =
      this->create_wall_timer(std::chrono::milliseconds(position_pub_interval),
                              std::bind(&Quad::positionPubCallback, this));
}

Quad::~Quad() {
  delete mavsdk_;
  delete action_;
  delete offboard_;
  delete telemetry_;
  delete passthrough_;
}

// methods
bool Quad::initialize(const std::string &port) {
  RCLCPP_INFO(this->get_logger(), "Initializing MavSDK interface.");
  // create connection
  mavsdk_ = new mavsdk::Mavsdk;
  mavsdk::ConnectionResult connection_result =
      mavsdk_->add_any_connection(port);
  if (connection_result != mavsdk::ConnectionResult::Success) {
    RCLCPP_ERROR(this->get_logger(),
                 "Initialisation failed: MavLink connection failed [%i].",
                 (int)connection_result);
    return false;
  }

  // connect to system
  system_ = get_system(*mavsdk_);
  if (!system_) {
    RCLCPP_ERROR(this->get_logger(),
                 "Initialisation failed: System connection failed.");
    return false;
  }

  // Instantiate plugins
  action_ = new mavsdk::Action{system_};
  offboard_ = new mavsdk::Offboard{system_};
  telemetry_ = new mavsdk::Telemetry{system_};
  passthrough_ = new mavsdk::MavlinkPassthrough{system_};

  initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
  return true;
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////// Services

void Quad::getStatus(
    std::shared_ptr<raptor_interface::srv::QuadStatus::Request> request,
    std::shared_ptr<raptor_interface::srv::QuadStatus::Response> response) {
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Received status request.");
  response->battery = telemetry_->battery().remaining_percent * 100.0;
  response->local_pos_ok = telemetry_->health().is_local_position_ok;
  response->armable = telemetry_->health().is_armable;
  // debug
  RCLCPP_INFO(this->get_logger(), "Sending back status: [%f, %i, %i]",
              response->battery, response->local_pos_ok, response->armable);
}

void Quad::arm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Received arming request.");
  const mavsdk::Action::Result arm_result = action_->arm();
  response->message = actionResultToString(arm_result);
  if (arm_result == mavsdk::Action::Result::Success) {
    response->success = true;
  } else {
    response->success = false;
  }
  // debug
  RCLCPP_INFO(this->get_logger(), "Arming result: [%s]",
              response->message.c_str());
}

void Quad::disarm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Received disarming request.");
  const mavsdk::Action::Result disarm_result = action_->disarm();
  response->message = actionResultToString(disarm_result);
  if (disarm_result == mavsdk::Action::Result::Success) {
    response->success = true;
  } else {
    response->success = false;
  }
  RCLCPP_INFO(this->get_logger(), "Disarming result: [%s]",
              response->message.c_str());
}

void Quad::startPosOffboard(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  RCLCPP_INFO(this->get_logger(),
              "Received position offboard starting request.");
  // send offboard message once before starting TODO
  mavsdk::Offboard::PositionNedYaw pos_msg{};
  pos_msg.north_m = telemetry_->position_velocity_ned().position.north_m;
  pos_msg.east_m = telemetry_->position_velocity_ned().position.east_m;
  pos_msg.down_m = telemetry_->position_velocity_ned().position.down_m;
  pos_msg.yaw_deg = 0.0; // TODO
  // starting offboard
  offboard_->set_position_ned(pos_msg);
  const mavsdk::Offboard::Result offboard_result = offboard_->start();
  response->message = offboardResultToString(offboard_result);
  offboard_->set_position_ned(pos_msg);
  if (offboard_result == mavsdk::Offboard::Result::Success) {
    is_offboard_ = true;
    response->success = true;
  } else {
    response->success = false;
  }
  RCLCPP_INFO(this->get_logger(),
              "Position offboard start result: [%s]",
              response->message.c_str());
}

void Quad::stopPosOffboard(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  RCLCPP_INFO(this->get_logger(),
              "Received position offboard stopping request.");
  const mavsdk::Offboard::Result offboard_result = offboard_->stop();
  response->message = offboardResultToString(offboard_result);
  if (offboard_result == mavsdk::Offboard::Result::Success) {
    is_offboard_ = false;
    response->success = true;
  } else {
    response->success = false;
  }
  RCLCPP_INFO(this->get_logger(),
              "Position offboard stop result: [%s]", response->message.c_str());
}

void Quad::takeoff(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Received takeoff request.");
  const mavsdk::Action::Result takeoff_result = action_->takeoff();
  response->message = actionResultToString(takeoff_result);
  if (takeoff_result == mavsdk::Action::Result::Success) {
    response->success = true;
  } else {
    response->success = false;
  }
  RCLCPP_INFO(this->get_logger(), "Takeoff result: [%s]",
              response->message.c_str());
}

void Quad::land(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Received land request.");
  const mavsdk::Action::Result land_result = action_->land();
  response->message = actionResultToString(land_result);
  if (land_result == mavsdk::Action::Result::Success) {
    response->success = true;
  } else {
    response->success = false;
  }
  RCLCPP_INFO(this->get_logger(), "Land result: [%s]",
              response->message.c_str());
}

void Quad::goToPos(
    std::shared_ptr<raptor_interface::srv::GoToPos::Request> request,
    std::shared_ptr<raptor_interface::srv::GoToPos::Response> response) {
  RCLCPP_INFO(this->get_logger(),
              "Received GoToPos request: [%f, %f, %f, %f]", request->x_ref,
              request->y_ref, request->z_ref, request->yaw_ref);
  // create message
  mavsdk::Offboard::PositionNedYaw pos_msg{};
  pos_msg.north_m = request->x_ref;
  pos_msg.east_m = -request->y_ref;
  pos_msg.down_m = -request->z_ref;
  pos_msg.yaw_deg = request->yaw_ref;
  // send message to quad
  offboard_->set_position_ned(pos_msg);

  response->success = true;
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////// Publishers

void Quad::positionPubCallback() {
  auto message = geometry_msgs::msg::Point();
  message.x = telemetry_->position_velocity_ned().position.north_m;
  message.y = -telemetry_->position_velocity_ned().position.east_m;
  message.z = -telemetry_->position_velocity_ned().position.down_m;
  // debug
  // RCLCPP_INFO(this->get_logger(), "Publishing Position: [%f,%f,%f]",
  // message.x, message.y, message.z);
  pub_position_->publish(message);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////// Helpers

std::string actionResultToString(mavsdk::Action::Result index) {
  std::string val[] = {"Unknown",
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
  return val[int(index)];
}

std::string offboardResultToString(mavsdk::Offboard::Result index) {
  std::string val[] = {"Unknown",         "Success",      "NoSystem",
                       "ConnectionError", "Busy",         "CommandDenied",
                       "Timeout",         "NoSetpointSet"};
  return val[int(index)];
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

std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk &mavsdk) {
  std::cout << "Waiting to discover system...\n";
  auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
  auto fut = prom.get_future();

  // We wait for new systems to be discovered, once we find one that has an
  // autopilot, we decide to use it.
  mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
    auto system = mavsdk.systems().back();

    if (system->has_autopilot()) {
      std::cout << "Discovered autopilot\n";

      // Unsubscribe again as we only want to find one system.
      mavsdk.subscribe_on_new_system(nullptr);
      prom.set_value(system);
    }
  });

  // We usually receive heartbeats at 1Hz, therefore we should find a
  // system after around 3 seconds max, surely.
  if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
    std::cerr << "No autopilot found.\n";
    return nullptr;
  }

  // Get discovered system now.
  return fut.get();
}
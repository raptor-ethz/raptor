#include "quad_control/mavsdk_wrapper.hpp"

MavsdkWrapper::MavsdkWrapper() {}


int MavsdkWrapper::initialize(const std::string& port)
{
  // create mavsdk instance
  mavsdk_ = std::make_shared<mavsdk::Mavsdk>();

  
  // create connection
  mavsdk::ConnectionResult connection_result = mavsdk_->add_any_connection(port);
  if (connection_result != mavsdk::ConnectionResult::Success) { return 1; } // Connection failed


  // connect to system
  system_ = get_system(*mavsdk_);
  if (!system_) { return 2; } // System connection failed


  // instantiate plugins
  action_ = std::make_shared<mavsdk::Action>(system_);
  offboard_ = std::make_shared<mavsdk::Offboard>(system_);
  telemetry_ = std::make_shared<mavsdk::Telemetry>(system_);
  passthrough_ = std::make_shared<mavsdk::MavlinkPassthrough>(system_);

  return 0;
}


int MavsdkWrapper::sendArmRequest() const
{
  return int(action_->arm());
}

int MavsdkWrapper::sendTakeoffRequest() const
{
  return int(action_->takeoff());
}

int MavsdkWrapper::sendLandRequest() const
{
  return int(action_->land());
}

int MavsdkWrapper::sendOffboardRequest() const
{
  return int(offboard_->start());
}

int MavsdkWrapper::sendPositionMessage (const std::array<float,3> &position, const float yaw) const
{
  // create message
  mavsdk::Offboard::PositionNedYaw pos_msg{}; // TODO should we instantiate the message on every function call or rather once as a resuable member variable?

  // fill message: transform from north-west-up to PX4's north-east-down
  pos_msg.north_m = position[0];
  pos_msg.east_m = -position[1];
  pos_msg.down_m = -position[2];
  pos_msg.yaw_deg = -yaw;

  // send message to px4
  return int(offboard_->set_position_ned(pos_msg)); // TODO is this thread safe?
}


bool MavsdkWrapper::isArmable() const
{
  return telemetry_->health().is_armable;
}

bool MavsdkWrapper::isLocalPositionOk() const
{
  return telemetry_->health().is_local_position_ok;
}
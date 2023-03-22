#include "quad_control/mavsdk_wrapper.hpp"

MavsdkWrapper::MavsdkWrapper(const std::shared_ptr<mavsdk::Mavsdk> &mavsdk,
                             const std::shared_ptr<mavsdk::System> &system,
                             const std::shared_ptr<mavsdk::Action> &action,
                             const std::shared_ptr<mavsdk::Offboard> &offboard,
                             const std::shared_ptr<mavsdk::Telemetry> &telemetry,
                             const std::shared_ptr<mavsdk::MavlinkPassthrough> &passthrough)
  : mavsdk_(mavsdk), system_(system), action_(action), offboard_(offboard), telemetry_(telemetry), passthrough_(passthrough)
{
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
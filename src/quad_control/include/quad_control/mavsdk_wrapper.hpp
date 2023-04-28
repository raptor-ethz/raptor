#pragma once

#include <iostream>
#include <future>

// mavsdk
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>


class MavsdkWrapper
{
public:
  MavsdkWrapper();
  ~MavsdkWrapper() {};

  int initialize(const std::string &port);

  int sendArmRequest() const;

  int sendTakeoffRequest() const;

  int sendLandRequest() const;

  // PRE: steady stream of offboard messages
  int sendOffboardRequest() const;

  int sendPositionMessage (const std::array<float,3> &position, const float yaw = 0) const;

  bool isArmable() const;

  bool isLocalPositionOk() const;

  // TODO get battery status

private:
  // mavsdk
  std::shared_ptr<mavsdk::Mavsdk> mavsdk_;
  std::shared_ptr<mavsdk::System> system_;
  std::shared_ptr<mavsdk::Action> action_;
  std::shared_ptr<mavsdk::Offboard> offboard_;
  std::shared_ptr<mavsdk::Telemetry> telemetry_;
  std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough_;
};




/**
 * Find PX4 flight controller.
 * 
 * @return Shared pointer to discovered system.
*/
std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk &mavsdk);
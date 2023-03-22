#pragma once

// mavsdk
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>


class MavsdkWrapper
{
public:
  MavsdkWrapper(const std::shared_ptr<mavsdk::Mavsdk> &mavsdk,
                const std::shared_ptr<mavsdk::System> &system,
                const std::shared_ptr<mavsdk::Action> &action,
                const std::shared_ptr<mavsdk::Offboard> &offboard,
                const std::shared_ptr<mavsdk::Telemetry> &telemetry,
                const std::shared_ptr<mavsdk::MavlinkPassthrough> &passthrough);
  ~MavsdkWrapper() {};

  int sendPositionMessage (const std::array<float,3> &position, const float yaw = 0) const;

private:
  // mavsdk
  std::shared_ptr<mavsdk::Mavsdk> mavsdk_;
  std::shared_ptr<mavsdk::System> system_;
  std::shared_ptr<mavsdk::Action> action_;
  std::shared_ptr<mavsdk::Offboard> offboard_;
  std::shared_ptr<mavsdk::Telemetry> telemetry_;
  std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough_;
};
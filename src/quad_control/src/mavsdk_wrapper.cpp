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
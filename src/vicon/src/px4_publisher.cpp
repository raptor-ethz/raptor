////////////////////////////////////////////////////////////////////////////////
// ROS

#include "rclcpp/rclcpp.hpp"
#include "raptor_interface/msg/pose.hpp"
#include "raptor_interface/msg/velocity.hpp"
#include "raptor_interface/msg/acceleration.hpp"

////////////////////////////////////////////////////////////////////////////////
// MavSDK, PX4

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;

////////////////////////////////////////////////////////////////////////////////
// MavSDK helper

void usage(const std::string &bin_name)
{
  std::cerr
      << "Usage : " << bin_name << " <connection_url>\n"
      << "Connection URL format should be :\n"
      << " For TCP : tcp://[server_host][:server_port]\n"
      << " For UDP : udp://[bind_host][:bind_port]\n"
      << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
      << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk &mavsdk)
{
  std::cout << "Waiting to discover system...\n";
  auto prom = std::promise<std::shared_ptr<System>>{};
  auto fut = prom.get_future();

  // We wait for new systems to be discovered, once we find one that has an
  // autopilot, we decide to use it.
  mavsdk.subscribe_on_new_system([&mavsdk, &prom]()
                                 {
    auto system = mavsdk.systems().back();

    if (system->has_autopilot()) {
      std::cout << "Discovered autopilot\n";

      // Unsubscribe again as we only want to find one system.
      mavsdk.subscribe_on_new_system(nullptr);
      prom.set_value(system);
    } });

  // We usually receive heartbeats at 1Hz, therefore we should find a
  // system after around 3 seconds max, surely.
  if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
  {
    std::cerr << "No autopilot found.\n";
    return {};
  }

  // Get discovered system now.
  return fut.get();
}

////////////////////////////////////////////////////////////////////////////////
// parameters

const int INTERVAL_RATE_HZ = 100;
const int INTERVAL_MS = 1000. / INTERVAL_RATE_HZ;
const int TELEMETRY_RATE_HZ = 150;

////////////////////////////////////////////////////////////////////////////////
// ROS publisher

class PX4Publisher : public rclcpp::Node
{
public:
  PX4Publisher(Telemetry* telemetry)
  : Node("px4_publisher"), telemetry_(telemetry)
  {
    pose_publisher_ = this->create_publisher<raptor_interface::msg::Pose>("px4_pose_nwu", 10);
    velocity_publisher_ = this->create_publisher<raptor_interface::msg::Velocity>("px4_vel_nwu", 10);
    acceleration_publisher_ = this->create_publisher<raptor_interface::msg::Acceleration>("px4_acc_flu", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(INTERVAL_MS), 
              std::bind(&PX4Publisher::publish, this));
  }

  void publish() {
    auto pose_msg = raptor_interface::msg::Pose();
    auto vel_msg = raptor_interface::msg::Velocity();
    auto acc_msg = raptor_interface::msg::Acceleration();

    const auto& pos_vel = telemetry_->position_velocity_ned();
    const auto& attitude_euler = telemetry_->attitude_euler();
    const auto& imu = telemetry_->imu();
    
    pose_msg.x_m = pos_vel.position.north_m;
    pose_msg.y_m = - pos_vel.position.east_m;
    pose_msg.z_m = - pos_vel.position.down_m;
    pose_msg.roll_deg = attitude_euler.roll_deg;
    pose_msg.pitch_deg = - attitude_euler.pitch_deg;
    pose_msg.yaw_deg = - attitude_euler.yaw_deg;

    vel_msg.x_m_s = pos_vel.velocity.north_m_s;
    vel_msg.y_m_s = - pos_vel.velocity.east_m_s;
    vel_msg.z_m_s = - pos_vel.velocity.down_m_s;

    acc_msg.x_m_s2 = imu.acceleration_frd.forward_m_s2;
    acc_msg.y_m_s2 = - imu.acceleration_frd.right_m_s2;
    acc_msg.z_m_s2 = - imu.acceleration_frd.down_m_s2;

    pose_publisher_->publish(pose_msg);
    velocity_publisher_->publish(vel_msg);
    acceleration_publisher_->publish(acc_msg);
  }

private:
  rclcpp::Publisher<raptor_interface::msg::Pose>::SharedPtr pose_publisher_;
  rclcpp::Publisher<raptor_interface::msg::Velocity>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<raptor_interface::msg::Acceleration>::SharedPtr acceleration_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  Telemetry *telemetry_;
};

////////////////////////////////////////////////////////////////////////////////
// main

int main(int argc, char *argv[])
{
  // check command line argument
  if (argc < 2)
  {
    usage(argv[0]);
    return 1;
  }

  // init mavsdk
  Mavsdk mavsdk;
  ConnectionResult connection_result =
      mavsdk.add_any_connection(argv[1]);

  if (connection_result != ConnectionResult::Success) {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }
  auto system = get_system(mavsdk);
  if (!system) {
    return 1;
  }

  // Instantiate plugins
  auto telemetry = Telemetry{system};

  // set telemetry subscription rate
  auto sub_rate_result = telemetry.set_rate_position_velocity_ned(TELEMETRY_RATE_HZ);
  if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
    std::cout << "Failed to set subscription rate for pos_vel: " << (int)sub_rate_result << std::endl;
    return 1;
  }
  sub_rate_result = telemetry.set_rate_attitude(TELEMETRY_RATE_HZ);
  if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
    std::cout << "Failed to set subscription rate for attitude: " << (int)sub_rate_result << std::endl;
    return 1;
  }
  sub_rate_result = telemetry.set_rate_imu(TELEMETRY_RATE_HZ);
  if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
    std::cout << "Failed to set subscription rate for imu: " << (int)sub_rate_result << std::endl;
    return 1;
  }
  std::cout << "Successfully set subscription rates for pos_vel, attitude, and imu." << std::endl;


  // init ros
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PX4Publisher>(&telemetry);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "PX4-telemetry publisher ready using the topics 'px4_pose_nwu', 'px4_vel_nwu' and 'px4_acc_flu'.");

  rclcpp::spin(node);

  rclcpp::shutdown();
}

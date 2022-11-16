#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/point.hpp"

// Todo make this into yaml file
const int pos_ref_pub_t = 50;
const int arm_delay = 5000;

class ReferenceGenerator : public rclcpp::Node
{
public:
  ReferenceGenerator() : Node("reference_generator"), count_(0)
  {
    pos_ref_ = {0.0, 0.0, 0.0};
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("position_ref", 10);

    client_arm_ = this->create_client<std_srvs::srv::Trigger>("arm");
    client_disarm_ = this->create_client<std_srvs::srv::Trigger>("disarm");
    client_takeoff_ = this->create_client<std_srvs::srv::Trigger>("takeoff");
    client_land_ = this->create_client<std_srvs::srv::Trigger>("land");
    client_start_offboard_ = this->create_client<std_srvs::srv::Trigger>("start_pos_offboard");
    client_stop_offboard_ = this->create_client<std_srvs::srv::Trigger>("stop_pos_offboard");

    timer_ = this->create_wall_timer( std::chrono::milliseconds(pos_ref_pub_t), 
                                      std::bind(&ReferenceGenerator::timer_callback, 
                                      this));
  }
  void setPos(double x, double y, double z)
  {
    pos_ref_.at(0) = x;
    pos_ref_.at(1) = y;
    pos_ref_.at(2) = z;
  }

  bool arm()
  {
    // check if service is available
    if (!client_arm_->wait_for_service(std::chrono::seconds(1))) {
      // TODO
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arming service not found.");
      return false;
    }
    // create request (empty)
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // send request
    auto result = client_arm_->async_send_request(request);
    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Quad successfully armed.");
      return true;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to arm quad.");
      return false;
    }
  }

  bool disarm()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_disarm_->async_send_request(request);
    // TODO return value.
  }

  bool takeoff()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_takeoff_->async_send_request(request);
    // TODO return value.
  }

  bool land()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_land_->async_send_request(request);
    // TODO return value.
  }

  bool startOffboard()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_start_offboard_->async_send_request(request);
    // TODO return value.
  }

  bool stopOffboard()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_stop_offboard_->async_send_request(request);
    // TODO return value.
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Point();
    message.x = pos_ref_.at(0);
    message.y = pos_ref_.at(1);
    message.z = pos_ref_.at(2);
    RCLCPP_INFO(this->get_logger(), "Publishing: [%f,%f,%f]", message.x, message.y, message.z);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_arm_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_disarm_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_takeoff_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_land_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_start_offboard_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_stop_offboard_;

  size_t count_;
  std::vector<double> pos_ref_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // start pos_ref publisher
  auto node = std::make_shared<ReferenceGenerator>();

  node->arm();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  rclcpp::shutdown();
  return 0;
}
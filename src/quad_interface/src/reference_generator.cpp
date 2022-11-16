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

    // timer_ = this->create_wall_timer( std::chrono::milliseconds(pos_ref_pub_t), 
    //                                   std::bind(&ReferenceGenerator::timer_callback, 
    //                                   this));
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
      // check response
      if (result.get()->success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Quad armed.");
        return true;
      }
      // mavsdk command failed
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to arm: %s", result.get()->message.c_str());
      return true;
    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arm service failed.");
      return false;
    }
  }

  bool disarm()
  {
    // check if service is available
    if (!client_disarm_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Disarming service not found.");
      return false;
    }
    // create request (empty)
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // send request
    auto result = client_disarm_->async_send_request(request);
    // wait until service completedtakeoff
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // check response
      if (result.get()->success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Quad disarmed.");
        return true;
      }
      // mavsdk command failed
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to disarm: %s", result.get()->message.c_str());
      return true;
    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Disarm service failed.");
      return false;
    }
  }

  bool takeoff()
  {
    // check if service is available
    if (!client_takeoff_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Takeoff service not found.");
      return false;
    }
    // create request (empty)
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // send request
    auto result = client_takeoff_->async_send_request(request);
    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // check response
      if (result.get()->success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Takeoff initiated.");
        return true;
      }
      // mavsdk command failed
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to takeoff: %s", result.get()->message.c_str());
      return true;
    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Takeoff service failed.");
      return false;
    }
  }

  bool land()
  {
    // check if service is available
    if (!client_land_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Land service not found.");
      return false;
    }
    // create request (empty)
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // send request
    auto result = client_land_->async_send_request(request);
    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // check response
      if (result.get()->success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Landing initiated.");
        return true;
      }
      // mavsdk command failed
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to land: %s", result.get()->message.c_str());
      return true;
    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Landing service failed.");
      return false;
    }
  }

  bool startOffboard()
  {
    // check if service is available
    if (!client_start_offboard_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Start offboard service not found.");
      return false;
    }
    // create request (empty)
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // send request
    auto result = client_start_offboard_->async_send_request(request);
    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // check response
      if (result.get()->success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started offboard.");
        return true;
      }
      // mavsdk command failed
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to start offboard: %s", result.get()->message.c_str());
      return true;
    } else {
      // service call failed
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Start offboard service failed.");
      return false;
    }
  }

  bool stopOffboard()
  {
    // check if service is available
    if (!client_stop_offboard_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Stop offboard service not found.");
      return false;
    }
    // create request (empty)
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // send request
    auto result = client_stop_offboard_->async_send_request(request);
    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      // check response
      if (result.get()->success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopped offboard.");
        return true;
      }
      // mavsdk command failed
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to stop offboard: %s", result.get()->message.c_str());
      return true;
    } else {
      // service call failed
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Stop offboard service failed.");
      return false;
    }
  }

  void goToPos()
  {
    auto message = geometry_msgs::msg::Point();
    message.x = pos_ref_.at(0);
    message.y = pos_ref_.at(1);
    message.z = pos_ref_.at(2);
    RCLCPP_INFO(this->get_logger(), "Publishing: [%f,%f,%f]", message.x, message.y, message.z);
    publisher_->publish(message);
    rclcpp::spin_some(this->get_node_base_interface());
  }
private:

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
  node->takeoff();
  std::this_thread::sleep_for(std::chrono::milliseconds(8000));

  // node->startOffboard();
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // node->setPos(2, 2, 2);
  // node->goToPos();
  // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  // node->setPos(0, 0, 1);
  // node->goToPos();
  // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  node->land();
  std::this_thread::sleep_for(std::chrono::milliseconds(8000));
  node->disarm();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  rclcpp::shutdown();
  return 0;
}
#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp" // ros2 client library
#include "std_msgs/msg/string.hpp" // include std_msg type
#include "std_srvs/srv/trigger.hpp"

// MAVSDK
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// helpers
#include "mavsdk/mavsdk_helper.h"

// class MinimalPublisher : public rclcpp::Node
// {
//   public:
//     MinimalPublisher()
//     : Node("minimal_publisher"), count_(0)
//     {
//       publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//       timer_ = this->create_wall_timer(
//       std::chrono::milliseconds(500), std::bind(&MinimalPublisher::timer_callback, this));
//     }

//   private:
//     void timer_callback()
//     {
//       auto message = std_msgs::msg::String();
//       message.data = "Hello, world! " + std::to_string(count_++);
//       RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//       publisher_->publish(message);
//     }
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//     size_t count_;
// };

class Quad : public rclcpp::Node
{
public:
  Quad(mavsdk::Action* action, mavsdk::Offboard* offboard, mavsdk::Telemetry* telemetry)
  : Node("quad_interface")
  {
    // init mavsdk variables
    action_ = action;
    offboard_ = offboard;
    telemetry_ = telemetry;

    // create services
    service_arm_ = this->create_service<std_srvs::srv::Trigger>("arm_quad", 
      std::bind(&Quad::arm, this, std::placeholders::_1, std::placeholders::_2));
    service_preflight_check_ = this->create_service<std_srvs::srv::Trigger>("preflight_check", 
      std::bind(&Quad::runPreflightCheck, this, std::placeholders::_1, std::placeholders::_2));
  }


private:
  void arm( std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received arming request");
    const mavsdk::Action::Result arm_result = action_->arm();  
    response->message = ActionResultToString(arm_result);
    if(arm_result == mavsdk::Action::Result::Success){
      response->success = true;
      
    }else{
      response->success = false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Arming result: [%s]",response->message.c_str());
  }

  void runPreflightCheck( std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received preflight check request");
    //if(request->data) response->success = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back preflicht response: [%d]", response->success);
  }
  
  // mavsdk
  mavsdk::Action* action_;
  mavsdk::Offboard* offboard_;
  mavsdk::Telemetry* telemetry_;

  // ros
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_arm_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_preflight_check_;
};




int main(int argc, char * argv[])
{ 
  /* INITIALIZE ROS*/
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<MinimalPublisher>());
  // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("quad_interface");


  // init mavsdk
  if (argc != 2)
  {
    usage(argv[0]);
    return 1;
  }

  mavsdk::Mavsdk mavsdk;
  mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

  if (connection_result != mavsdk::ConnectionResult::Success)
  {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }

  auto system = get_system(mavsdk);
  if (!system)
  {
    return 1;
  }

  auto action = mavsdk::Action{system};              // for arming / disarming etc
  auto offboard = mavsdk::Offboard{system};          // for offboard control
  auto telemetry = mavsdk::Telemetry{system};        // for telemetry services

  auto node = std::make_shared<Quad>(&action, &offboard, &telemetry);

  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
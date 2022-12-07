#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp" // ros2 client library
// custom ros interface
#include "raptor_interface/msg/pose.hpp"


// parameters TODO
const int PUBLISHER_INTERVAL_MS = 1;


class ViconPublisher : public rclcpp::Node
{
public:
  ViconPublisher()
  : Node("vicon_publisher"), count_(0)
  {
    publisher_ =
      this->create_publisher<raptor_interface::msg::Pose>("pose", 10);
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(PUBLISHER_INTERVAL_MS),
      std::bind(&ViconPublisher::publisherCallback, this));
  }

private:
  void publisherCallback() 
  {
    auto message = raptor_interface::msg::Pose();
    message.x = 1;

    // DEBUG
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing: [%f]", message.x);
    publisher_->publish(message);
  }

  rclcpp::Publisher<raptor_interface::msg::Pose>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};



int main(int argc, char *argv[])
{
  // initialize ros
  rclcpp::init(argc, argv);

  // create node
  auto node = std::make_shared<ViconPublisher>();

  // inform
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vicon interface server is ready.");

  // spin node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
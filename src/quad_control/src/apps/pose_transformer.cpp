#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/pose.hpp"
#include "raptor_interface/msg/pose.hpp"


class PoseTransformer : public rclcpp::Node
{
public:
  PoseTransformer() : Node("pose_transformer_node")
  {
    // create subscirber to std_msgs::msg::Pose 
    sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/slam/odometry", 10, std::bind(&PoseTransformer::poseCallback, this, std::placeholders::_1));
    // create publisher
    pub_ = this->create_publisher<raptor_interface::msg::Pose>("slam_pose_nwu", 10);
  }

private:
  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr received_msg)
  {
    raptor_interface::msg::Pose outgoing_msg;
    outgoing_msg.x_m = received_msg->position.x;
    outgoing_msg.y_m = received_msg->position.y;
    outgoing_msg.z_m = received_msg->position.z;

    // convert the incoming quaternion to roll, pitch, yaw
    tf2::Quaternion q(
      received_msg->orientation.x,
      received_msg->orientation.y,
      received_msg->orientation.z,
      received_msg->orientation.w);
    tf2::Matrix3x3 m(q); 

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    outgoing_msg.roll_deg = roll * 180.0 / M_PI;
    outgoing_msg.pitch_deg = pitch * 180.0 / M_PI;
    outgoing_msg.yaw_deg = yaw * 180.0 / M_PI;

    // publish message
    pub_->publish(outgoing_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;
  rclcpp::Publisher<raptor_interface::msg::Pose>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};




int main(int argc, char *argv[])
{
  // initialize ros
  rclcpp::init(argc, argv);

  // initialize the node
  auto transformer_node = std::make_shared<PoseTransformer>();

  // info msg that node is running
  RCLCPP_INFO(transformer_node->get_logger(), "Optimus Prime ready for action.");
  // run the node until ros is not ok
  rclcpp::spin(transformer_node);

  rclcpp::shutdown();
  return 0;
}
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
// std interface
#include "geometry_msgs/msg/point.hpp" // TODO
#include "std_srvs/srv/trigger.hpp"
// custom interface
#include "raptor_interface/srv/go_to_pos.hpp"
#include "raptor_interface/srv/quad_status.hpp"
#include "raptor_interface/srv/set_servo.hpp"
#include "raptor_interface/msg/pose.hpp"

// Todo make this into yaml file
const int pos_ref_pub_t = 50;
const int arm_delay = 5000;
std::string object_name = "mocap_srl_box";

class ReferenceGenerator : public rclcpp::Node {
public:
  ReferenceGenerator() : Node("reference_generator") {
    pos_ref_ = {0.0, 0.0, 2.0};
    // initialize service clients
    client_quad_status_ =
        this->create_client<raptor_interface::srv::QuadStatus>("quad_status");
    client_arm_ = this->create_client<std_srvs::srv::Trigger>("arm");
    client_disarm_ = this->create_client<std_srvs::srv::Trigger>("disarm");
    client_takeoff_ = this->create_client<std_srvs::srv::Trigger>("takeoff");
    client_land_ = this->create_client<std_srvs::srv::Trigger>("land");
    client_start_offboard_ =
        this->create_client<std_srvs::srv::Trigger>("start_offboard");
    client_stop_offboard_ =
        this->create_client<std_srvs::srv::Trigger>("stop_offboard");
    client_go_to_pos_ =
        this->create_client<raptor_interface::srv::GoToPos>("go_to_pos");
    client_set_right_gripper_ =
        this->create_client<raptor_interface::srv::SetServo>(
            "rightGripper_deg");
    client_set_left_gripper_ =
        this->create_client<raptor_interface::srv::SetServo>("leftGripper_deg");
    // initialize subscribers
    subscriber_mocap_object_ = 
        this->create_subscription<raptor_interface::msg::Pose>(
            object_name, 
            10, 
            std::bind(&ReferenceGenerator::mocapObjectCallback, 
                      this, 
                      std::placeholders::_1));
  }

  //  TODO
  void setPos(double x, double y, double z) {
    pos_ref_.at(0) = x;
    pos_ref_.at(1) = y;
    pos_ref_.at(2) = z;
  }

  int doPreflightCheck() {
    // check if service is available
    if (!client_quad_status_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Quad Status service not found.");
      return 404; // ros service not found
    }

    // create request (empty)
    auto request =
        std::make_shared<raptor_interface::srv::QuadStatus::Request>();
    // send request
    auto result = client_quad_status_->async_send_request(request);

    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // get response from future
      auto response = result.get();
      // TODO do preflight check here
      return 0;

    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Preflight Check service failed.");
      return 401; // ros error
    }
  }

  int arm() {
    // check if service is available
    if (!client_arm_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arming service not found.");
      return 404; // ros service not found
    }

    // create request (empty)
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // send request
    auto result = client_arm_->async_send_request(request);

    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // get response from future
      auto response = result.get();

      // check response
      if (!(response->success)) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to arm: [%s]",
                    response->message.c_str());
        return 301; // mavsdk command failed
      }

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Quad armed.");
      // wait for propellers to stabilise
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      return 0; // success

    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Arm service failed.");
      return 401; // ros error
    }
  }

  int disarm() {
    // check if service is available
    if (!client_disarm_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Disarming service not found.");
      return 404; // ros service not found
    }

    // create request (empty)
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // send request
    auto result = client_disarm_->async_send_request(request);

    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // get response from future
      auto response = result.get();

      // check response
      if (!(response->success)) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to disarm: [%s]",
                    response->message.c_str());
        return 301; // mavsdk command failed
      }

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Quad disarmed.");
      return 0; // success

    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Disarm service failed.");
      return 401; // ros error
    }
  }

  int takeoff() {
    // check if service is available
    if (!client_takeoff_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Takeoff service not found.");
      return 404; // ros service not found
    }

    // create request (empty)
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // send request
    auto result = client_takeoff_->async_send_request(request);

    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // get response from future
      auto response = result.get();

      // check response
      if (!(response->success)) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to takeoff: [%s]",
                    response->message.c_str());
        return 301; // mavsdk command failed
      }

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Takeoff initiated.");
      // wait to reach sufficient height (TODO)
      std::this_thread::sleep_for(std::chrono::milliseconds(8000));
      return 0; // success

    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Takeoff service failed.");
      return 401; // ros error
    }
  }

  int land() {
    // check if service is available
    if (!client_land_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Land service not found.");
      return 404; // ros service not found
    }

    // create request (empty)
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // send request
    auto result = client_land_->async_send_request(request);

    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // get response from future
      auto response = result.get();

      // check response
      if (!(response->success)) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to land: [%s]",
                    response->message.c_str());
        return 301; // mavsdk command failed
      }

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Landing initiated.");
      // wait for quad to land TODO
      std::this_thread::sleep_for(std::chrono::milliseconds(10000));
      return 0; // success

    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Landing service failed.");
      return 401; // ros error
    }
  }

  int startOffboard() {
    // check if service is available
    if (!client_start_offboard_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Start offboard service not found.");
      return 404; // ros service not found
    }

    // create request (empty)
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // send request
    auto result = client_start_offboard_->async_send_request(request);

    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // get response from future
      auto response = result.get();

      // check response
      if (!(response->success)) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Failed to start offboard: [%s]",
                    response->message.c_str());
        return 301; // mavsdk command failed
      }

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Offboard started.");
      // wait for drone to stabilize
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      return 0; // success

    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Offboard service failed.");
      return 401; // ros error
    }
  }

  int stopOffboard() {
    // check if service is available
    if (!client_stop_offboard_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Stop offboard service not found.");
      return 404; // ros service not found
    }

    // create request (empty)
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // send request
    auto result = client_stop_offboard_->async_send_request(request);

    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // get response from future
      auto response = result.get();

      // check response
      if (!(response->success)) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Failed to stop offboard: [%s]", response->message.c_str());
        return 301; // mavsdk command failed
      }

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Offboard stopped.");
      return 0; // success

    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Offboard service failed.");
      return 401; // ros error
    }
  }

  int goToPos(float x_ref, float y_ref, float z_ref, float yaw_ref,
               int time_ms) {
    // check if service is available
    if (!client_go_to_pos_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "GoToPos service not found.");
      return false;
    }

    // TODO check reference

    // create request
    auto request = std::make_shared<raptor_interface::srv::GoToPos::Request>();
    request->x_ref = x_ref;
    request->y_ref = y_ref;
    request->z_ref = z_ref;
    request->yaw_ref = yaw_ref;
    RCLCPP_INFO(this->get_logger(), "Requesting GoToPos: [%f,%f,%f,%f]", x_ref,
                y_ref, z_ref, yaw_ref);

    // send request
    auto result = client_go_to_pos_->async_send_request(request);

    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // get response from future
      auto response = result.get();
      // check response
      if (!(response->success)) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "GoToPos failed: [%s]",
                    response->message.c_str());
        return 301; // mavsdk command denied
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GoToPos sent.");
      // TODO wait
      std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
      return 0; // success

    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "GoToPos service failed.");
      return 401; // ros error
    }
  }

  int goToObject(float x_offset, float y_offset, float z_offset, float yaw_ref,
               int time_ms) {
    // check if service is available
    if (!client_go_to_pos_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "GoToPos service not found.");
      return false;
    }

    // TODO check reference

    // create request
    auto request = std::make_shared<raptor_interface::srv::GoToPos::Request>();
    request->x_ref = object_x + x_offset;
    request->y_ref = object_y + y_offset;
    request->z_ref = object_z + z_offset;
    request->yaw_ref = yaw_ref;
    RCLCPP_INFO(this->get_logger(), "Requesting GoToPos: [%f,%f,%f,%f]", 
        request->x_ref,
        request->y_ref,
        request->z_ref,
        yaw_ref);

    // send request
    auto result = client_go_to_pos_->async_send_request(request);

    // wait until service completed
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // get response from future
      auto response = result.get();
      // check response
      if (!(response->success)) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "GoToPos failed: [%s]",
                    response->message.c_str());
        return 301; // mavsdk command denied
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GoToPos sent.");
      // TODO wait
      std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
      return 0; // success

    } else {
      // service call unsuccessfull
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "GoToPos service failed.");
      return 401; // ros error
    }
  }

  void setGripperAngle(double left, double right) {

    // create requests (empty)
    auto request_left =
        std::make_shared<raptor_interface::srv::SetServo::Request>();
    auto request_right =
        std::make_shared<raptor_interface::srv::SetServo::Request>();

    request_left->angle = left;
    request_right->angle = right;

    client_set_right_gripper_->async_send_request(request_right);
    client_set_left_gripper_->async_send_request(request_left);
  }

  void mocapObjectCallback(const raptor_interface::msg::Pose::SharedPtr msg) {
    // DEBUG
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
    //     "Callback called with position [%f,%f,%f].",
    //     msg.x,
    //     msg.y,
    //     msg.z);
    object_x = msg->x_m;
    object_y = -msg->y_m; // TODO
    object_z = msg->z_m;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

  // service clients
  rclcpp::Client<raptor_interface::srv::QuadStatus>::SharedPtr
      client_quad_status_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_arm_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_disarm_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_takeoff_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_land_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_start_offboard_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_stop_offboard_;
  rclcpp::Client<raptor_interface::srv::GoToPos>::SharedPtr client_go_to_pos_;
  rclcpp::Client<raptor_interface::srv::SetServo>::SharedPtr
      client_set_right_gripper_;
  rclcpp::Client<raptor_interface::srv::SetServo>::SharedPtr
      client_set_left_gripper_;

  // subscriber clients
  rclcpp::Subscription<raptor_interface::msg::Pose>::SharedPtr subscriber_mocap_object_;

  std::vector<double> pos_ref_;

  // object
  float object_x;
  float object_y;
  float object_z;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // start pos_ref publisher
  auto node = std::make_shared<ReferenceGenerator>();

  if (node->doPreflightCheck()) {
    exit(0);
  }

  // TODO
  // exit(0);

  if (node->arm()) {
    exit(0);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  node->takeoff();

  node->startOffboard();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // position demo
  node->goToPos(1,0,0.5,0,4000);
  node->goToPos(1,-1,0.5,0,4000);
  node->goToPos(0,-1,0.5,0,4000);

  // GRASP DEMO
  // node->goToObject(0,0,0.4,0,4000);
  // node->goToObject(0,0,0.18,0,2000);

  // node->setGripperAngle(10.0, 10.0);
  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // node->goToObject(0,0,0.6,0,4000);
  // node->setGripperAngle(90.0, 90.0);
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // node->goToPos(1, 1, 1, 0, 4000);
  // node->goToPos(1, 0, 1, 0, 4000);
  // node->goToPos(0, 0, 1, 0, 4000);
  // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  // node->goToPos(0,0,1,0,4000);


  // go home
  node->goToPos(0,0,0.5,0,4000);
  node->goToPos(0,0,0.2,0,2000);

  node->land();

  node->disarm();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  rclcpp::shutdown();
  return 0;
}
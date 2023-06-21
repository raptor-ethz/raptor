#include "gripper_control/gripper_control.hpp"




// #define SERIAL_PORT "/dev/ttyACM2" TODO

Gripper::Gripper(const std::string &port) : Node("gripper_control") {
  using namespace std::placeholders;

  RCLCPP_INFO(this->get_logger(), "Initializing...");

  // serial connection - serial port: /dev/ttyXXXX
  int serialResult = serial_.openDevice(port.c_str(), 115200);
  if (serialResult == -2) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial connection to %s", port.c_str());
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Successful serial connection.");

  // service servers
  srv_set_gripper_ = this->create_service<SetGripper>(
    "set_gripper", std::bind(&Gripper::setGripper, this, _1, _2));
}

void Gripper::setGripper(const std::shared_ptr<SetGripper::Request> request,
                          std::shared_ptr<SetGripper::Response> response)
{
  // check if requested values are between 0 and 90
  if (request->left_angle_deg < 0 || request->left_angle_deg > 90 ||
      request->right_angle_deg < 0 || request->right_angle_deg > 90) {
    RCLCPP_ERROR(this->get_logger(), "Gripper angles must be between 0 and 90 degrees.");
    response->success = 0;
    return;
  }
  
  cmd_[0] = request->left_angle_deg;
  cmd_[1] = request->right_angle_deg;
  serial_.writeBytes(&cmd_, sizeof(cmd_));
  response->success = 1;
  RCLCPP_INFO(this->get_logger(), "Set gripper to [L: %d, R: %d].", cmd_[0], cmd_[1]);
}






// TODO old code



// void set_right_angle(
//     std::shared_ptr<raptor_interface::srv::SetServo::Request> request,
//     std::shared_ptr<raptor_interface::srv::SetServo::Response> response) {
//   cmd[0] = request->angle;
//   response->success = true;
// }

// void set_left_angle(
//     std::shared_ptr<raptor_interface::srv::SetServo::Request> request,
//     std::shared_ptr<raptor_interface::srv::SetServo::Response> response) {
//   cmd[1] = request->angle;
//   response->success = true;
// }

// // command line argument: serial port, i.e. /dev/ttyUSB0
// int main(int argc, char **argv) {
//   return 0;
// }

//   if (argc != 2) {
//     std::cout << "ERROR: no serial port given as input argument" << std::endl;
//     return 0;
//   }
//   std::string serial_port = argv[1];
//   std::cout << "Initializing serial connection to port " << serial_port
//             << " ..." << std::endl;





//   // initialize ROS
//   rclcpp::init(argc, argv);

//   std::shared_ptr<rclcpp::Node> node =
//       rclcpp::Node::make_shared("gripper_interface_node");

//   auto leftGripper_service =
//       node->create_service<raptor_interface::srv::SetServo>("rightGripper_deg",
//                                                             &set_right_angle);
//   auto rightGripper_service =
//       node->create_service<raptor_interface::srv::SetServo>("leftGripper_deg",
//                                                             &set_left_angle);
//   for (;;) {
//     rclcpp::spin_some(node);
//     serial.writeBytes(&cmd, sizeof(cmd));
//   }

//   rclcpp::shutdown();

//   return 0;
// }

// // serialib
// #include "serialib.h"

// #define SERIAL_PORT "/dev/ttyACM2"

// using std::chrono::milliseconds;
// using std::chrono::seconds;
// using std::this_thread::sleep_for;

// int main(int argc, char *argv[]) {

//   if (argc != 2) {
//     std::cout << "ERROR: no serial port given as input argument" <<
//     std::endl; return 0;
//   }
//   std::string serial_port = argv[1];
//   std::cout << "Initializing serial connection to port " << serial_port
//             << " ..." << std::endl;

//   // initialize Serial object
//   serialib serial;

//   // If connection fails, return the error code otherwise, display a success
//   // message
//   int serialResult = serial.openDevice(serial_port.c_str(), 115200);
//   if (serialResult == -2) {
//     std::cout << "Could not initialize serial connection to " << serial_port
//               << std::endl;
//     return -1;
//   }
//   std::cout << "Successful connection to " << serial_port << std::endl;

//   // initialize Data
//   unsigned char Command_Data[] = {0};

//   for (unsigned char i = 0; i < 180; i += 45) {
//     /* SERIAL SENDING */

//     //  write bytes (8bit = cnversion to char type)
//     Command_Data[0] = i;

//     serial.writeBytes(&Command_Data, sizeof(Command_Data));
//     std::cout << "just wrote" << (int)Command_Data[0] << std::endl;

//     std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   }
//   serial.closeDevice();
//   return 0;
// }
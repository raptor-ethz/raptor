#include "rclcpp/rclcpp.hpp"
#include "gripper_control/gripper_control.hpp"




/**
 * Print error message when no port is provided as argument. TODO
*/
// void usage(const std::string &bin_name) {
//   std::cerr
//       << "Usage : " << bin_name << " <connection_url>\n"
//       << "Connection URL format should be :\n"
//       << " For TCP : tcp://[server_host][:server_port]\n"
//       << " For UDP : udp://[bind_host][:bind_port]\n"
//       << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
//       << "For example, to connect to the simulator use URL: udp://:14540\n";
// }


int main(int argc, char *argv[])
{
  // check command line input
  if (argc < 2)
  {
    std::cout << "ERROR: no serial port given as input argument" << std::endl;
    return 1;
    // usage(argv[0]);
  }

  // initialize ros
  rclcpp::init(argc, argv);

  // initialize node
  auto gripper_control_node = std::make_shared<Gripper>(argv[1]);

  // spin node if initialization was successful
  if(rclcpp::ok()) {
    RCLCPP_INFO(gripper_control_node->get_logger(), "Ready.");
    rclcpp::spin(gripper_control_node);
  }

  //   serial.closeDevice(); TODO necessary?
  rclcpp::shutdown();
  return 0;
}
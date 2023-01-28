#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "u3Streamer.h"
#include <iostream>

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("u3_streamming");

    auto u3_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("/force_torque_sensor_raw/u3", 1000);

    u3Streamer u3;

    if (!u3.isInit())
    {
        std::cout << "U3 initialzation unsuccessful" << std::endl;
        return -1;
    }
    // rclcpp::Rate loop_rate(250);
    std::cout<< "read Lab Jack started" << std::endl;

    int count = 0;
    while (rclcpp::ok())
    {
        std::array<double, NumChannels> voltages;
        u3.getStreamData(voltages);
        // u3.stream();  
        // std::cout << voltages[0] << " " << voltages[1] << " " << voltages[2] << " " << voltages[3] << std::endl;
        std_msgs::msg::Float32MultiArray msg;
        msg.data.assign(voltages.data(), voltages.data() + NumChannels);
        u3_pub->publish(msg);
        rclcpp::spin_some(node);

        // Sleep seems to cause issues with buffer reading
        // limit the speed in getStreamData function with numReadsPerDisplay
        // loop_rate.sleep();   
        ++count;
    }

    return 0;
}
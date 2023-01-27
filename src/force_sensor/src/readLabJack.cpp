#include "rclcpp/rclcpp.hpp"
// #include "ros/ros.h"
// #include "soft_robot_hand/FloatVector.h"
#include "raptor_interface/msg/force.hpp"
#include "u3Streamer.h"
#include <iostream>

int main(int argc, char **argv)
{

    // ros::init(argc, argv, "u3_streamming");
    rclcpp::init(argc, argv);

    // ros::NodeHandle n;
    auto node = rclcpp::Node::make_shared("u3_streamming");

    // ros::Publisher u3_pub = n.advertise<soft_robot_hand::FloatVector>("/force_torque_sensor_raw/u3", 1000);
    auto u3_pub = node->create_publisher<raptor_interface::msg::Force>("/force_torque_sensor_raw/u3", 1000);

    u3Streamer u3;

    if (!u3.isInit())
    {
        std::cout << "U3 initialzation unsuccessful" << std::endl;
        return -1;
    }
    // ros::Rate loop_rate(250);
    std::cout<< "read Lab Jack started" << std::endl;

    int count = 0;
    while (rclcpp::ok())
    {
        std::array<double, NumChannels> voltages;
        u3.getStreamData(voltages);
        // u3.stream();  
        raptor_interface::msg::Force msg;
        msg.data.assign(voltages.data(), voltages.data() + NumChannels);
        u3_pub->publish(msg);
        // ros::spinOnce();
        rclcpp::spin_some(node);

        // Sleep seems to cause issues with buffer reading
        // limit the speed in getStreamData function with numReadsPerDisplay
        // loop_rate.sleep();   
        ++count;
    }

    return 0;
}
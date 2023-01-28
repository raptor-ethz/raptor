#ifndef FTCONVERTER_H
#define FTCONVERTER_H

#include <stdio.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ftconfig.h"
#include "std_msgs/msg/float32_multi_array.hpp"

class FTConverter: public rclcpp::Node {
public:
    FTConverter(char *calfilepath);
    ~FTConverter();

    bool biasInit = false;

    void initBias();
    // int setTransfromation(std::array<float, 6>& transform);
    void getMeasurement(std::array<float, 6> &measurement);

    void u3Callback(std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void adcCallback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

private:
    // std::string calfilepath_;  // path to the calibration files
    Calibration *cal_; // struct containing calibration information
    short sts_;        // return value from functions

    std::array<float, 6> voltages_{0, 0, 0, 0, 0, 0};       // array to store raw measurments
    std::array<float, 6> bias_{0, 0, 0, 0, 0, 0};           // array to store bias
    std::array<float, 6> transformation_{0, 0, 0, 0, 0, 0}; // transform includes a translation along the Z-axis and a rotation about the X-axis.

    std::mutex voltage_mtx_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr u3Sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr adcSub;
};

#endif
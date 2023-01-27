#ifndef FTCONVERTER_H
#define FTCONVERTER_H

#include <stdio.h>
#include <string>

// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
#include "ftconfig.h"

// #include "soft_robot_hand/FloatVector.h"
#include "raptor_interface/msg/force.hpp"

class FTConverter
{
public:
    FTConverter(char *calfilepath);
    ~FTConverter();

    bool biasInit = false;

    void initBias();
    // int setTransfromation(std::array<float, 6>& transform);
    void getMeasurement(std::array<float, 6> &measurement);

    void u3Callback(raptor_interface::msg::Force msg);
    void adcCallback(raptor_interface::msg::Force msg);

private:
    // std::string calfilepath_;  // path to the calibration files
    Calibration *cal_; // struct containing calibration information
    short sts_;        // return value from functions

    std::array<float, 6> voltages_{0, 0, 0, 0, 0, 0};       // array to store raw measurments
    std::array<float, 6> bias_{0, 0, 0, 0, 0, 0};           // array to store bias
    std::array<float, 6> transformation_{0, 0, 0, 0, 0, 0}; // transform includes a translation along the Z-axis and a rotation about the X-axis.

    std::mutex voltage_mtx_;
};

#endif
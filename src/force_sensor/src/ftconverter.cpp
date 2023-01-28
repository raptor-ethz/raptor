#include <stdio.h>
#include <filesystem>
#include "ftconverter.h"

FTConverter::FTConverter(char *calfilepath) : Node("FT_Converter") {
    u3Sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/force_torque_sensor_raw/u3", 1000, std::bind(&FTConverter::u3Callback, this, std::placeholders::_1));
    adcSub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/force_torque_sensor_raw/adc", 1000, std::bind(&FTConverter::adcCallback, this, std::placeholders::_1));

    // create Calibration struct
    cal_ = createCalibration(calfilepath, 1);
    if (cal_ == NULL) {
        printf("\nSpecified calibration could not be loaded.\n");
        return;
    }

    transformation_ = std::array<float, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Set force units.
    // This step is optional; by default, the units are inherited from the calibration file.
    sts_ = SetForceUnits(cal_, const_cast<char*>("N"));
    switch (sts_) {
    case 0:
        break; // successful completion
    case 1:
        printf("Invalid Calibration struct");
        return;
    case 2:
        printf("Invalid force units");
        return;
    default:
        printf("Unknown error");
        return;
    }

    // Set torque units.
    // This step is optional; by default, the units are inherited from the calibration file.
    sts_ = SetTorqueUnits(cal_, const_cast<char*>("N-m"));
    switch (sts_) {
    case 0:
        break; // successful completion
    case 1:
        printf("Invalid Calibration struct");
        return;
    case 2:
        printf("Invalid torque units");
        return;
    default:
        printf("Unknown error");
        return;
    }

    // Set tool transform.
    // This line is only required if you want to move or rotate the sensor's coordinate system.
    // This example tool transform translates the coordinate system 20 mm along the Z-axis
    // and rotates it 45 degrees about the X-axis.
    sts_ = SetToolTransform(cal_, transformation_.data(), const_cast<char*>("mm"), const_cast<char*>("degrees"));
    switch (sts_) {
    case 0:
        break; // successful completion
    case 1:
        printf("Invalid Calibration struct");
        return;
    case 2:
        printf("Invalid distance units");
        return;
    case 3:
        printf("Invalid angle units");
        return;
    default:
        printf("Unknown error");
        return;
    }

}

FTConverter::~FTConverter() {
    destroyCalibration(cal_);
}

void FTConverter::initBias() {
    voltage_mtx_.lock();
    bias_ = voltages_;
    voltage_mtx_.unlock();
    biasInit = true;

    Bias(cal_, bias_.data());

    std::cout << "Bias is initialized to be : [ " ;
    for (auto i : bias_)
    {
        std::cout << i << ", ";
    }
    std::cout << " ]" << std::endl;
}

void FTConverter::getMeasurement(std::array<float, 6> &measurement) {
    if (biasInit)
        ConvertToFT(cal_, voltages_.data(), measurement.data());
    else
        std::cout << "bias not initialized, please initialize bias first" << std::endl;
}

void FTConverter::u3Callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    voltage_mtx_.lock();
    for (int i = 0; i < 4; i++)
    {
        // save reading from u3 (SG2, SG3, SG4, SG5)
        voltages_.at(i + 2) = msg->data[i];
    }
    voltages_.at(1) = 0.0;

    voltage_mtx_.unlock();
}

void FTConverter::adcCallback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data[0] > 2. || msg->data[0] < -2.) {
        std::cout << "ADC is saturated, please adjust the measurement range" << std::endl;
        return ;
    }
    voltage_mtx_.lock();
    // use adc to read first channel (SG0)
    voltages_.at(0) = msg->data[0];
    voltage_mtx_.unlock();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::filesystem::path path = std::filesystem::current_path();
    std::string calPath = path.string() + "/src/force_sensor/config/FT7724.cal";
    std::cout << calPath << std::endl;

    auto ft = std::make_shared<FTConverter>((char *)calPath.c_str());
    auto u3_pub = ft->create_publisher<std_msgs::msg::Float32MultiArray>("/force_torque_sensor/readings", 1000);

    int count = 0;
    rclcpp::Rate loop_rate(890);
    while (rclcpp::ok()) {

        rclcpp::spin_some(ft);
        if (count == 2000) {
            ft->initBias();
            break;
        }
        count++;

        loop_rate.sleep();
    }

    while (rclcpp::ok()) {
        std::array<float, 6> ForceTorque;
        ft->getMeasurement(ForceTorque);
        std_msgs::msg::Float32MultiArray msg;
        msg.data.assign(ForceTorque.data(), ForceTorque.data() + 6);

        u3_pub->publish(msg);

        rclcpp::spin_some(ft);
        loop_rate.sleep();
    }

    return 0;
}
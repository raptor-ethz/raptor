#include <stdio.h>
#include "ftconverter.h"

// #include <ros/package.h>
// #include "rclcpp/rclcpp.hpp"

FTConverter::FTConverter(char *calfilepath)
{
    // create Calibration struct
    cal_ = createCalibration(calfilepath, 1);
    if (cal_ == NULL)
    {
        printf("\nSpecified calibration could not be loaded.\n");
        return;
    }

    transformation_ = std::array<float, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Set force units.
    // This step is optional; by default, the units are inherited from the calibration file.
    sts_ = SetForceUnits(cal_, "N");
    switch (sts_)
    {
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
    sts_ = SetTorqueUnits(cal_, "N-m");
    switch (sts_)
    {
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
    sts_ = SetToolTransform(cal_, transformation_.data(), "mm", "degrees");
    switch (sts_)
    {
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

    // ros::NodeHandle nh;
}

FTConverter::~FTConverter()
{
    destroyCalibration(cal_);
}

void FTConverter::initBias()
{
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

void FTConverter::getMeasurement(std::array<float, 6> &measurement)
{
    if (biasInit)
        ConvertToFT(cal_, voltages_.data(), measurement.data());
    else
        std::cout << "bias not initialized, please initialize bias first" << std::endl;
}

void FTConverter::u3Callback(raptor_interface::msg::Force msg)
{
    voltage_mtx_.lock();
    for (int i = 0; i < 4; i++)
    {
        // save reading from u3 (SG2, SG3, SG4, SG5)
        voltages_.at(i + 2) = msg.data[i];
    }
    voltages_.at(1) = 0.0;

    voltage_mtx_.unlock();
}

void FTConverter::adcCallback(raptor_interface::msg::Force msg)
{
    if (msg.data[0] > 2. || msg.data[0] < -2.)
    {
        std::cout << "ADC is saturated, please adjust the measurement range" << std::endl;
        return ;
    }
    voltage_mtx_.lock();
    // use adc to read first channel (SG0)
    voltages_.at(0) = msg.data[0];
    voltage_mtx_.unlock();
}

// class FTConverterNode : public rclcpp::Node {
//     public:
//     FTConverter(char *calfilepath) : Node("FT_Converter") {
//         // create Calibration struct
//         cal_ = createCalibration(calfilepath, 1);
//         if (cal_ == NULL)
//         {
//             printf("\nSpecified calibration could not be loaded.\n");
//             return;
//         }

//         transformation_ = std::array<float, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//         // Set force units.
//         // This step is optional; by default, the units are inherited from the calibration file.
//         sts_ = SetForceUnits(cal_, "N");
//         switch (sts_)
//         {
//         case 0:
//             break; // successful completion
//         case 1:
//             printf("Invalid Calibration struct");
//             return;
//         case 2:
//             printf("Invalid force units");
//             return;
//         default:
//             printf("Unknown error");
//             return;
//         }

//         // Set torque units.
//         // This step is optional; by default, the units are inherited from the calibration file.
//         sts_ = SetTorqueUnits(cal_, "N-m");
//         switch (sts_)
//         {
//         case 0:
//             break; // successful completion
//         case 1:
//             printf("Invalid Calibration struct");
//             return;
//         case 2:
//             printf("Invalid torque units");
//             return;
//         default:
//             printf("Unknown error");
//             return;
//         }

//         // Set tool transform.
//         // This line is only required if you want to move or rotate the sensor's coordinate system.
//         // This example tool transform translates the coordinate system 20 mm along the Z-axis
//         // and rotates it 45 degrees about the X-axis.
//         sts_ = SetToolTransform(cal_, transformation_.data(), "mm", "degrees");
//         switch (sts_)
//         {
//         case 0:
//             break; // successful completion
//         case 1:
//             printf("Invalid Calibration struct");
//             return;
//         case 2:
//             printf("Invalid distance units");
//             return;
//         case 3:
//             printf("Invalid angle units");
//             return;
//         default:
//             printf("Unknown error");
//             return;
//         }

//         // initialize subscribers
//         subscriber_mocap_object_ = 
//             this->create_subscription<raptor_interface::msg::Pose>(
//                 object_name, 
//                 10, 
//                 std::bind(&ReferenceGenerator::mocapObjectCallback, 
//                         this, 
//                         std::placeholders::_1));
//     }

//     ~FTConverter() {
//         destroyCalibration(cal_);
//     }

//     initBias() {
//         voltage_mtx_.lock();
//         bias_ = voltages_;
//         voltage_mtx_.unlock();
//         biasInit = true;

//         Bias(cal_, bias_.data());

//         std::cout << "Bias is initialized to be : [ " ;
//         for (auto i : bias_)
//         {
//             std::cout << i << ", ";
//         }
//         std::cout << " ]" << std::endl;
//     }

//     getMeasurement(std::array<float, 6> &measurement) {
//         if (biasInit)
//             ConvertToFT(cal_, voltages_.data(), measurement.data());
//         else
//             std::cout << "bias not initialized, please initialize bias first" << std::endl;
//     }

//     u3Callback(raptor_interface::msg::Force msg) {
//         voltage_mtx_.lock();
//         for (int i = 0; i < 4; i++)
//         {
//             // save reading from u3 (SG2, SG3, SG4, SG5)
//             voltages_.at(i + 2) = msg.data[i];
//         }
//         voltages_.at(1) = 0.0;

//         voltage_mtx_.unlock();
//     }

//     adcCallback(raptor_interface::msg::Force msg) {
//         if (msg.data[0] > 2. || msg.data[0] < -2.)
//         {
//             std::cout << "ADC is saturated, please adjust the measurement range" << std::endl;
//             return ;
//         }
//         voltage_mtx_.lock();
//         // use adc to read first channel (SG0)
//         voltages_.at(0) = msg.data[0];
//         voltage_mtx_.unlock();
//     }

//     private:
//     // subscriber clients
//     rclcpp::Subscription<raptor_interface::msg::Pose>::SharedPtr subscriber_mocap_object_;
// };

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // std::string path = ros::package::getPath("soft_robot_hand");
    std::string calPath = "../config/FT7724.cal";
    FTConverter ft((char *)calPath.c_str());

    // ros::NodeHandle nh;
    auto node = rclcpp::Node::make_shared("FT_Converter");

    // ros::Publisher u3_pub = nh.advertise<soft_robot_hand::FloatVector>("/force_torque_sensor/readings", 1000);
    auto u3_pub = node->create_publisher<raptor_interface::msg::Force>("/force_torque_sensor/readings", 1000);
    
    // ros::Subscriber u3Sub = nh.subscribe("/force_torque_sensor_raw/u3", 1000, &FTConverter::u3Callback, &ft);
    // ros::Subscriber adcSub = nh.subscribe("/force_torque_sensor_raw/adc", 1000, &FTConverter::adcCallback, &ft);

    // auto u3Sub = node->create_subscription("/force_torque_sensor_raw/u3", 1000, std::bind(&FTConverter::u3Callback, &ft, _1));
    // auto adcSub = node->create_subscription("/force_torque_sensor_raw/adc", 1000, std::bind(&FTConverter::u3Callback, &ft, _1))

    int count = 0;
    rclcpp::Rate loop_rate(890);
    while (rclcpp::ok())
    {
        // ros::spinOnce();
        rclcpp::spin_some(node);

        count++;
        if (count == 2000)
        {
            ft.initBias();
            break;
        }
        loop_rate.sleep();
    }

    while (rclcpp::ok())
    {
        std::array<float, 6> ForceTorque;
        ft.getMeasurement(ForceTorque);
        raptor_interface::msg::Force msg;
        msg.data.assign(ForceTorque.data(), ForceTorque.data() + 6);
        u3_pub->publish(msg);
        // ros::spinOnce();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    return 0;
}
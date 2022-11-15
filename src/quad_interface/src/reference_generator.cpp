#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/point.hpp"

// Todo make this into yaml file
const int pos_ref_pub_t = 50;
const int arm_delay = 5000;

class ReferenceGenerator : public rclcpp::Node
{
public:
    ReferenceGenerator()
        : Node("reference_generator"), count_(0)
    {
        pos_ref_ = {0.0, 0.0, 0.0};
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("position_ref", 10);

        arm_client_ = this->create_client<std_srvs::srv::Trigger>("arm");
        disarm_client_ = this->create_client<std_srvs::srv::Trigger>("disarm");
        takeoff_client_ = this->create_client<std_srvs::srv::Trigger>("takeoff");
        land_client_ = this->create_client<std_srvs::srv::Trigger>("land");
        start_offboard_client_ = this->create_client<std_srvs::srv::Trigger>("start_pos_offboard");
        stop_offboard_client_ = this->create_client<std_srvs::srv::Trigger>("stop_pos_offboard");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(pos_ref_pub_t), std::bind(&ReferenceGenerator::timer_callback, this));
    }
    void set_pos(double x, double y, double z)
    {
        pos_ref_.at(0) = x;
        pos_ref_.at(1) = y;
        pos_ref_.at(2) = z;
    }

    bool arm()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = arm_client_->async_send_request(request);
        // TODO return value.
    }

    bool disarm()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = disarm_client_->async_send_request(request);
        // TODO return value.
    }

    bool takeoff()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = takeoff_client_->async_send_request(request);
        // TODO return value.
    }

    bool land()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = land_client_->async_send_request(request);
        // TODO return value.
    }

    bool start_offboard()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = start_offboard_client_->async_send_request(request);
        // TODO return value.
    }

    bool stop_offboard()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = stop_offboard_client_->async_send_request(request);
        // TODO return value.
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Point();
        message.x = pos_ref_.at(0);
        message.y = pos_ref_.at(1);
        message.z = pos_ref_.at(2);
        RCLCPP_INFO(this->get_logger(), "Publishing: [%f,%f,%f]", message.x, message.y, message.z);
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr disarm_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr takeoff_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr land_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_offboard_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_offboard_client_;

    size_t count_;
    std::vector<double> pos_ref_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // start pos_ref publisher
    auto node = std::make_shared<ReferenceGenerator>();

    node->arm();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    node->takeoff();
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    node->start_offboard();

    rclcpp::spin(node);
    // TODO better timing etc and error handling
    node->stop_offboard();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    node->land();
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    node->disarm();
    // auto result = client->async_send_request(request);

    // // Wait for the result.
    // if (rclcpp::spin_until_future_complete(node, result) ==
    //     rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    // }
    // else
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    // }

    rclcpp::shutdown();
    return 0;
}
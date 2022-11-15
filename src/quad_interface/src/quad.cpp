#include "quad.hpp"

Quad::Quad(mavsdk::Action *action, mavsdk::Offboard *offboard, mavsdk::Telemetry *telemetry) : Node("quad_interface")
{
    // init mavsdk variables
    action_ = action;
    offboard_ = offboard;
    telemetry_ = telemetry;

    // create services
    service_arm_ = this->create_service<std_srvs::srv::Trigger>("arm",
                                                                std::bind(&Quad::arm, this, std::placeholders::_1, std::placeholders::_2));
    service_preflight_check_ = this->create_service<std_srvs::srv::Trigger>("preflight_check",
                                                                            std::bind(&Quad::runPreflightCheck, this, std::placeholders::_1, std::placeholders::_2));
    service_disarm_ = this->create_service<std_srvs::srv::Trigger>("disarm",
                                                                   std::bind(&Quad::disarm, this, std::placeholders::_1, std::placeholders::_2));
    service_takeoff_ = this->create_service<std_srvs::srv::Trigger>("takeoff",
                                                                    std::bind(&Quad::takeoff, this, std::placeholders::_1, std::placeholders::_2));
    service_land_ = this->create_service<std_srvs::srv::Trigger>("land",
                                                                 std::bind(&Quad::takeoff, this, std::placeholders::_1, std::placeholders::_2));
}

void Quad::arm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received arming request");
    const mavsdk::Action::Result arm_result = action_->arm();
    response->message = this->actionResultToString(arm_result);
    if (arm_result == mavsdk::Action::Result::Success)
    {
        response->success = true;
    }
    else
    {
        response->success = false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Arming result: [%s]", response->message.c_str());
}

void Quad::disarm(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received disarming request");
    const mavsdk::Action::Result disarm_result = action_->disarm();
    response->message = this->actionResultToString(disarm_result);
    if (disarm_result == mavsdk::Action::Result::Success)
    {
        response->success = true;
    }
    else
    {
        response->success = false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disarming result: [%s]", response->message.c_str());
}

void Quad::takeoff(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received takeoff request");
    const mavsdk::Action::Result takeoff_result = action_->takeoff();
    response->message = this->actionResultToString(takeoff_result);
    if (takeoff_result == mavsdk::Action::Result::Success)
    {
        response->success = true;
    }
    else
    {
        response->success = false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Takeoff result: [%s]", response->message.c_str());
}

void Quad::land(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received land request");
    const mavsdk::Action::Result land_result = action_->land();
    response->message = this->actionResultToString(land_result);
    if (land_result == mavsdk::Action::Result::Success)
    {
        response->success = true;
    }
    else
    {
        response->success = false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Land result: [%s]", response->message.c_str());
}

void Quad::runPreflightCheck(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received preflight check request");
    // if(request->data) response->success = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back preflicht response: [%d]", response->success);
}

std::string Quad::actionResultToString(mavsdk::Action::Result index)
{
    std::string val[] =
        {"Unknown", "Success", "NoSystem", "ConnectionError", "Busy", "CommandDenied", "CommandDeniedLandedStateUnknown", "CommandDeniedNotLanded", "Timeout", "VtolTransitionSupportUnknown", "NoVtolTransitionSupport", "ParameterError", "Unsupported"};
    return val[int(index)];
}

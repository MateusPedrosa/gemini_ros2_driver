#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "gemini_ros2/msg/sonar_status.hpp"

class GeminiSonarNode : public rclcpp::Node {
public:
    GeminiSonarNode();
    ~GeminiSonarNode();

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<gemini_ros2::msg::SonarStatus>::SharedPtr status_pub_;

    void onGeminiMessageReceived(unsigned int msgType, size_t size, const char* const value);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    void applyInitialConfigurations(); 
};

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "gemini_ros2/msg/sonar_status.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "GenesisSerializer/CompassRecord.h"

class GeminiSonarNode : public rclcpp::Node {
public:
    GeminiSonarNode();
    ~GeminiSonarNode();

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      image_pub_;
    rclcpp::Publisher<gemini_ros2::msg::SonarStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr        imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr        imu_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr  gps_pub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  play_pause_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  play_stop_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  record_srv_;

    void onGeminiMessageReceived(unsigned int msgType, size_t size, const char* const value);
    void publishImu(const GLF::CompassDataRecord* pRec);

    void playbackPauseCallback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                               std_srvs::srv::SetBool::Response::SharedPtr res);
    void playbackStopCallback(const std_srvs::srv::Trigger::Request::SharedPtr req,
                              std_srvs::srv::Trigger::Response::SharedPtr res);
    void recordCallback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                        std_srvs::srv::SetBool::Response::SharedPtr res);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    void applyInitialConfigurations();
};

#include "gemini_ros2/gemini_sonar_node.hpp"

#include "Svs5Seq/Svs5SequencerApi.h"
#include "Gemini/GeminiStructuresPublic.h"
#include "GenesisSerializer/GlfApi.h"
#include "GenesisSerializer/GenericDataTypes.h"
#include "GenesisSerializer/GpsRecord.h"

#include <optional>
#include <vector>
#include <string>
#include <functional>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

GeminiSonarNode::GeminiSonarNode() : Node("gemini_sonar_node") {
    // Initialize your publishers
    image_pub_   = this->create_publisher<sensor_msgs::msg::Image>("sonar/image", 10);
    status_pub_  = this->create_publisher<gemini_ros2::msg::SonarStatus>("sonar/status", 10);
    imu_pub_     = this->create_publisher<sensor_msgs::msg::Imu>("sonar/imu", 10);
    imu_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("sonar/imu_raw", 10);
    mag_pub_     = this->create_publisher<sensor_msgs::msg::MagneticField>("sonar/mag", 10);
    gps_pub_     = this->create_publisher<sensor_msgs::msg::NavSatFix>("sonar/gps", 10);

    // Services
    play_pause_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "sonar/playback/pause",
        std::bind(&GeminiSonarNode::playbackPauseCallback, this, std::placeholders::_1, std::placeholders::_2));
    play_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "sonar/playback/stop",
        std::bind(&GeminiSonarNode::playbackStopCallback, this, std::placeholders::_1, std::placeholders::_2));
    record_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "sonar/record",
        std::bind(&GeminiSonarNode::recordCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Declare ROS 2 parameters for environment toggling
    this->declare_parameter<bool>("live_mode", false);
    this->declare_parameter<std::string>("log_file_path", "/workspace/build/sim_test/sample.glf");

    // Declare all acoustic parameters so the launch file can set them
    this->declare_parameter<double>("range", 20.0);
    this->declare_parameter<int>("gain", 50);
    this->declare_parameter<int>("ping_mode", 0);
    this->declare_parameter<int>("frequency_mode", 0);
    this->declare_parameter<double>("range_threshold", 20.0);
    this->declare_parameter<bool>("high_resolution", true);
    this->declare_parameter<int>("chirp_mode", 2);
    this->declare_parameter<bool>("use_manual_sos", false);
    this->declare_parameter<double>("manual_sos", 1500.0);
    this->declare_parameter<bool>("sonar_inverted", false);
    this->declare_parameter<double>("aperture", 120.0);
    this->declare_parameter<int>("image_quality", 3);
    this->declare_parameter<bool>("loop_playback", false);
    this->declare_parameter<int>("playback_speed", 1);
    this->declare_parameter<std::string>("record_path", "");

    // Bind the live parameter listener
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&GeminiSonarNode::parametersCallback, this, std::placeholders::_1));

    // Bind the callback to this specific class instance
    SequencerApi::StartSvs5(std::bind(&GeminiSonarNode::onGeminiMessageReceived, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // Retrieve the parameters
    bool is_live = this->get_parameter("live_mode").as_bool();
    std::string log_path = this->get_parameter("log_file_path").as_string();

    // Environment Routing
    if (is_live) {
        // --- LIVE NETWORK MODE ---
        bool fOnline = true;
        SequencerApi::Svs5SetConfiguration(
            SequencerApi::SVS5_CONFIG_ONLINE,
            sizeof(bool),
            &fOnline,
            0 
        );
        RCLCPP_INFO(this->get_logger(), "Gemini ROS 2 Node Started. Connecting to LIVE sonar hardware...");
    } 
    else {
        // --- LOG PLAYBACK MODE ---
        std::vector<std::string> filesList;
        filesList.push_back(log_path);
        SequencerApi::ListOfFileNames listOfFileNames(filesList);

        SequencerApi::Svs5SetConfiguration(
            SequencerApi::SVS5_CONFIG_PLAY_START,
            sizeof(listOfFileNames),
            &listOfFileNames,
            0
        );
        RCLCPP_INFO(this->get_logger(), "Gemini ROS 2 Node Started. Playing back log file: %s", log_path.c_str());
    }

    std::string record_path = this->get_parameter("record_path").as_string();
    if (!record_path.empty()) {
        SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_FILE_LOCATION,
            record_path.size() + 1, record_path.c_str(), 0);
    }

    applyInitialConfigurations();
}

GeminiSonarNode::~GeminiSonarNode() {
    SequencerApi::StopSvs5();
}

void GeminiSonarNode::onGeminiMessageReceived(unsigned int msgType, size_t size, const char* const value) {
    switch (msgType) { 
        case SequencerApi::TGT_IMG_PLAYBACK:
        {
            GLF::GenesisPlaybackTargetImage* playbackImg = (GLF::GenesisPlaybackTargetImage*)value;
            if (!playbackImg || !playbackImg->m_pLogTgtImage) break;
            const auto& mi = playbackImg->m_pLogTgtImage->m_mainImage;
            if (!mi.m_uiEndBearing || !mi.m_uiEndRange || !mi.m_vecData || mi.m_vecData->empty()) break;

            auto img_msg = sensor_msgs::msg::Image();
            img_msg.header.stamp    = this->now();
            img_msg.header.frame_id = "sonar_link";
            img_msg.width    = mi.m_uiEndBearing - mi.m_uiStartBearing;
            img_msg.height   = mi.m_uiEndRange;
            img_msg.encoding = "mono8";
            img_msg.step     = img_msg.width;
            img_msg.data.assign(mi.m_vecData->begin(), mi.m_vecData->end());

            image_pub_->publish(img_msg);
            RCLCPP_INFO(this->get_logger(), "Published playback frame %d", playbackImg->m_frame);
        }
        break;

        case SequencerApi::GLF_LIVE_TARGET_IMAGE:
        {
            GLF::GLogTargetImage* liveImg = (GLF::GLogTargetImage*)value;
            if (!liveImg) break;
            const auto& mi = liveImg->m_mainImage;
            if (!mi.m_uiEndBearing || !mi.m_uiEndRange || !mi.m_vecData || mi.m_vecData->empty()) break;

            auto img_msg = sensor_msgs::msg::Image();
            img_msg.header.stamp    = this->now();
            img_msg.header.frame_id = "sonar_link";
            img_msg.width    = mi.m_uiEndBearing - mi.m_uiStartBearing;
            img_msg.height   = mi.m_uiEndRange;
            img_msg.encoding = "mono8";
            img_msg.step     = img_msg.width;
            img_msg.data.assign(mi.m_vecData->begin(), mi.m_vecData->end());

            image_pub_->publish(img_msg);
        }
        break;

        case SequencerApi::GEMINI_STATUS:
        {
            const GLF::GeminiSonarStatusMessage* const statusMsg =
                (const GLF::GeminiSonarStatusMessage* const)value;
            const GLF::GeminiStatusRecord& status = statusMsg->m_geminiSonarStatus;

            gemini_ros2::msg::SonarStatus status_msg;
            status_msg.header.stamp = this->now();
            status_msg.source_device = status.m_deviceID;

            status_msg.shutdown_status           = status.m_shutdownStatus;
            status_msg.over_temperature_shutdown = (status.m_shutdownStatus & 0x0001);
            status_msg.out_of_water_shutdown     = (status.m_shutdownStatus & 0x0002);

            status_msg.bf_die_temp   = status.m_dieT;
            status_msg.bf_pcb_temp   = status.m_vgaT1;
            status_msg.bf_comms_temp = status.m_vgaT2;
            status_msg.bf_tx_temp    = status.m_txT;
            status_msg.bf_psu_temp   = status.m_psuT;

            status_msg.da_pcb_temp      = status.m_vgaT3;
            status_msg.da_afe0_top_temp = status.m_afe0TopTemp;
            status_msg.da_afe0_bot_temp = status.m_afe0BotTemp;
            status_msg.da_afe1_top_temp = status.m_afe1TopTemp;
            status_msg.da_afe1_bot_temp = status.m_afe1BotTemp;
            status_msg.da_afe2_top_temp = status.m_afe2TopTemp;
            status_msg.da_afe2_bot_temp = status.m_afe2BotTemp;
            status_msg.da_afe3_top_temp = status.m_afe3TopTemp;
            status_msg.da_afe3_bot_temp = status.m_afe3BotTemp;

            status_pub_->publish(status_msg);
        }
        break;

        case SequencerApi::COMPASS_RECORD:
        {
            GLF::GLogV4ReplyMessage* gnsV4ReplyMsg = (GLF::GLogV4ReplyMessage*)value;
            uint8_t dataType = gnsV4ReplyMsg->m_header.m_ciHeader.m_dataType;
            if (dataType == 99)
            {
                std::vector<unsigned char>& vecCompass = *gnsV4ReplyMsg->m_v4GenericRec.m_vecData;
                publishImu(reinterpret_cast<const GLF::CompassDataRecord*>(vecCompass.data()));
            }
            else
            {
                RCLCPP_WARN_ONCE(this->get_logger(), "COMPASS_RECORD: unsupported dataType %d (only structured type 99 is handled)", dataType);
            }
        }
        break;

        case SequencerApi::AHRS_HPR_DATA:
        {
            publishImu(reinterpret_cast<const GLF::CompassDataRecord*>(value));
        }
        break;

        case SequencerApi::GPS_RECORD:
        {
            GLF::GLogV4ReplyMessage* gnsV4ReplyMsg = (GLF::GLogV4ReplyMessage*)value;
            if (gnsV4ReplyMsg->m_header.m_ciHeader.m_dataType != 99) break;
            std::vector<unsigned char>& vec = *gnsV4ReplyMsg->m_v4GenericRec.m_vecData;
            GLF::GpsDataRecord* pGps = (GLF::GpsDataRecord*)vec.data();

            auto gps_msg = sensor_msgs::msg::NavSatFix();
            gps_msg.header.stamp    = this->now();
            gps_msg.header.frame_id = "sonar_link";
            // bits 1 (lat valid) and 2 (lon valid) of m_gpsValid indicate a position fix
            gps_msg.status.status   = ((pGps->m_gpsValid & 0x06) == 0x06)
                                        ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                                        : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            gps_msg.latitude        = pGps->m_llRec.m_latDegrees;
            gps_msg.longitude       = pGps->m_llRec.m_longDegrees;
            gps_msg.altitude        = pGps->m_llRec.m_altitude;
            gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
            gps_pub_->publish(gps_msg);
        }
        break;

        case SequencerApi::AHRS_RAW_DATA:
        {
            const auto* pRaw = reinterpret_cast<const GLF::AHRSRawDataRecord*>(value);

            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp    = this->now();
            imu_msg.header.frame_id = "sonar_link";
            imu_msg.angular_velocity.x    = pRaw->m_gyroX;
            imu_msg.angular_velocity.y    = pRaw->m_gyroY;
            imu_msg.angular_velocity.z    = pRaw->m_gyroZ;
            imu_msg.linear_acceleration.x = pRaw->m_accelX;
            imu_msg.linear_acceleration.y = pRaw->m_accelY;
            imu_msg.linear_acceleration.z = pRaw->m_accelZ;
            imu_msg.orientation_covariance[0] = -1.0; // orientation not available in raw data
            imu_raw_pub_->publish(imu_msg);

            auto mag_msg = sensor_msgs::msg::MagneticField();
            mag_msg.header       = imu_msg.header;
            mag_msg.magnetic_field.x = pRaw->m_magX; // SDK units: normalised, not Tesla
            mag_msg.magnetic_field.y = pRaw->m_magY;
            mag_msg.magnetic_field.z = pRaw->m_magZ;
            mag_pub_->publish(mag_msg);
        }
        break;

        default:
            // Handle other messages if necessary
            break;
    }
}

void GeminiSonarNode::publishImu(const GLF::CompassDataRecord* pRec) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Publishing IMU data (heading=%.2f pitch=%.2f roll=%.2f)", pRec->m_heading, pRec->m_pitch, pRec->m_roll);
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "sonar_link";

    tf2::Quaternion q;
    q.setRPY(
        pRec->m_roll    * (M_PI / 180.0),
        pRec->m_pitch   * (M_PI / 180.0),
        pRec->m_heading * (M_PI / 180.0)
    );

    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

    imu_pub_->publish(imu_msg);
}

// Applies all configurations when the node first boots
void GeminiSonarNode::applyInitialConfigurations() {
    std::vector<rclcpp::Parameter> initial_params;
    initial_params.push_back(this->get_parameter("range"));
    initial_params.push_back(this->get_parameter("gain"));
    initial_params.push_back(this->get_parameter("ping_mode"));
    initial_params.push_back(this->get_parameter("frequency_mode"));
    initial_params.push_back(this->get_parameter("range_threshold"));
    initial_params.push_back(this->get_parameter("high_resolution"));
    initial_params.push_back(this->get_parameter("chirp_mode"));
    initial_params.push_back(this->get_parameter("use_manual_sos"));
    initial_params.push_back(this->get_parameter("manual_sos"));
    initial_params.push_back(this->get_parameter("sonar_inverted"));
    initial_params.push_back(this->get_parameter("aperture"));
    initial_params.push_back(this->get_parameter("image_quality"));
    initial_params.push_back(this->get_parameter("loop_playback"));
    initial_params.push_back(this->get_parameter("playback_speed"));

    parametersCallback(initial_params);
}

// Intercepts live terminal changes and routes them to the Gemini API
rcl_interfaces::msg::SetParametersResult GeminiSonarNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters applied successfully";

    // Pre-scan for paired parameters. The callback fires before ROS 2 commits the new
    // values, so get_parameter() returns stale data for the sibling of a pair. Caching
    // new values here ensures both members of a pair use their updated values even when
    // they arrive in the same batch.
    std::optional<int>    new_freq_mode;
    std::optional<double> new_range_thresh;
    std::optional<bool>   new_use_manual_sos;
    std::optional<double> new_manual_sos;
    for (const auto& p : parameters) {
        if      (p.get_name() == "frequency_mode")  new_freq_mode      = p.as_int();
        else if (p.get_name() == "range_threshold") new_range_thresh   = p.as_double();
        else if (p.get_name() == "use_manual_sos")  new_use_manual_sos = p.as_bool();
        else if (p.get_name() == "manual_sos")      new_manual_sos     = p.as_double();
    }

    bool freq_applied = false;
    bool sos_applied  = false;

    for (const auto &param : parameters) {
        if (param.get_name() == "range") {
            double range = param.as_double();
            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_RANGE, sizeof(double), &range, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: Range -> %.2f m", range);
        }
        else if (param.get_name() == "gain") {
            int gain = param.as_int();
            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_GAIN, sizeof(int), &gain, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: Gain -> %d%%", gain);
        }
        else if (param.get_name() == "high_resolution") {
            bool high_res = param.as_bool();
            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_HIGH_RESOLUTION, sizeof(bool), &high_res, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: High Resolution -> %s", high_res ? "Enabled" : "Disabled");
        }
        else if (param.get_name() == "chirp_mode") {
            CHIRP_MODE chirp = static_cast<CHIRP_MODE>(param.as_int());
            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_CHIRP_MODE, sizeof(CHIRP_MODE), &chirp, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: Chirp Mode Configured");
        }
        else if (param.get_name() == "frequency_mode" || param.get_name() == "range_threshold") {
            if (freq_applied) continue;
            freq_applied = true;

            RangeFrequencyConfig freq_config;
            freq_config.m_frequency      = static_cast<FREQUENCY>(new_freq_mode.value_or(this->get_parameter("frequency_mode").as_int()));
            freq_config.m_rangeThreshold = new_range_thresh.value_or(this->get_parameter("range_threshold").as_double());

            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_RANGE_RESOLUTION, sizeof(RangeFrequencyConfig), &freq_config, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: 1200iK Frequency Mode Configured");
        }
        else if (param.get_name() == "ping_mode") {
            SequencerApi::SequencerPingMode ping_config;
            int p_mode = param.as_int();

            ping_config.m_bFreeRun      = (p_mode == 0);
            ping_config.m_extTTLTrigger = (p_mode == 2);
            ping_config.m_manualTrigger = (p_mode == 3);
            if (p_mode == 1) ping_config.m_msInterval = 100;

            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_PING_MODE, sizeof(SequencerApi::SequencerPingMode), &ping_config, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: Ping Mode Configured");
        }
        else if (param.get_name() == "use_manual_sos" || param.get_name() == "manual_sos") {
            if (sos_applied) continue;
            sos_applied = true;

            SequencerApi::SequencerSosConfig sos_config;
            sos_config.m_bUsedUserSos = new_use_manual_sos.value_or(this->get_parameter("use_manual_sos").as_bool());
            sos_config.m_manualSos    = static_cast<float>(new_manual_sos.value_or(this->get_parameter("manual_sos").as_double()));

            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_SOUND_VELOCITY, sizeof(SequencerApi::SequencerSosConfig), &sos_config, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: Speed of Sound Configured");
        }
        else if (param.get_name() == "sonar_inverted") {
            auto orient = param.as_bool()
                ? SequencerApi::SONAR_ORIENTATION_DOWN
                : SequencerApi::SONAR_ORIENTATION_UP;
            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_SONAR_ORIENTATION,
                sizeof(SequencerApi::ESvs5SonarOrientation), &orient, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: Sonar Orientation -> %s", param.as_bool() ? "Inverted" : "Upright");
        }
        else if (param.get_name() == "aperture") {
            double ap = param.as_double();
            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_APERTURE, sizeof(double), &ap, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: Aperture -> %.0f deg", ap);
        }
        else if (param.get_name() == "image_quality") {
            SequencerApi::SonarImageQualityLevel q;
            q.m_performance  = static_cast<SequencerApi::ESdkPerformanceControl>(param.as_int());
            q.m_screenPixels = 2048;
            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_CPU_PERFORMANCE,
                sizeof(SequencerApi::SonarImageQualityLevel), &q, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: Image Quality -> %d", param.as_int());
        }
        else if (param.get_name() == "loop_playback") {
            bool loop = param.as_bool();
            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_PLAY_REPEAT, sizeof(bool), &loop, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: Loop Playback -> %s", loop ? "Enabled" : "Disabled");
        }
        else if (param.get_name() == "playback_speed") {
            int speed = param.as_int();
            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_PLAY_SPEED, sizeof(int), &speed, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: Playback Speed -> %s", speed == 0 ? "Free-run" : "Real-time");
        }
    }
    return result;
}

void GeminiSonarNode::playbackPauseCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res)
{
    bool pause = req->data;
    SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_PLAY_PAUSE, sizeof(bool), &pause, 0);
    res->success = true;
    res->message = pause ? "Playback paused" : "Playback resumed";
}

void GeminiSonarNode::playbackStopCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_PLAY_STOP, 0, nullptr, 0);
    res->success = true;
    res->message = "Playback stopped";
}

void GeminiSonarNode::recordCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res)
{
    bool record = req->data;
    SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_REC, sizeof(bool), &record, 0);
    res->success = true;
    res->message = record ? "Recording started" : "Recording stopped";
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeminiSonarNode>());
    rclcpp::shutdown();
    return 0;
}

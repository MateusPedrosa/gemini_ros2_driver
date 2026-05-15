#include "gemini_ros2/gemini_sonar_node.hpp"

#include "Svs5Seq/Svs5SequencerApi.h"
#include "Gemini/GeminiStructuresPublic.h"
#include "GenesisSerializer/GlfApi.h"

#include <vector>
#include <string>
#include <functional>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

GeminiSonarNode::GeminiSonarNode() : Node("gemini_sonar_node") {
    // Initialize your publishers
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("sonar/image", 10);
    status_pub_ = this->create_publisher<gemini_ros2::msg::SonarStatus>("sonar/status", 10);

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

    // Push the initial parameter states from the launch file to the sonar
    applyInitialConfigurations();

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
}

GeminiSonarNode::~GeminiSonarNode() {
    SequencerApi::StopSvs5();
}

void GeminiSonarNode::onGeminiMessageReceived(unsigned int msgType, size_t size, const char* const value) {
    switch (msgType) { 
        case SequencerApi::TGT_IMG_PLAYBACK:
        {
            GLF::GenesisPlaybackTargetImage* playbackImg = (GLF::GenesisPlaybackTargetImage*)value;

            auto img_msg = sensor_msgs::msg::Image();
            img_msg.header.stamp = this->now();
            img_msg.header.frame_id = "sonar_link";

            img_msg.width = playbackImg->m_pLogTgtImage->m_mainImage.m_uiEndBearing;
            img_msg.height = playbackImg->m_pLogTgtImage->m_mainImage.m_uiEndRange;
            img_msg.encoding = "mono8";
            img_msg.step = img_msg.width;

            auto& raw_data = playbackImg->m_pLogTgtImage->m_mainImage.m_vecData;
            img_msg.data.assign(raw_data->begin(), raw_data->end());

            image_pub_->publish(img_msg);

            RCLCPP_INFO(this->get_logger(), "Published playback frame %d", playbackImg->m_frame);
        }
        break;

        case SequencerApi::GEMINI_STATUS:
        {
            // First, cast to the base network message to check the source device ID
            const CGemNetMsg* const pNetMsg = (const CGemNetMsg* const)value;

            gemini_ros2::msg::SonarStatus status_msg;
            status_msg.header.stamp = this->now();
            status_msg.source_device = pNetMsg->m_head.m_src_sub_device_id;

            // Helper lambda to decode 12-bit signed MK2 temperatures
            auto decode_mk2_temp = [](unsigned short raw_temp) -> double {
                signed short temp = raw_temp & 0x0FFF; 
                if (raw_temp & 0x0800) {               
                    temp |= 0xF000;                    
                }
                return static_cast<double>(temp) * 0.0625;
            };

            // Device ID 2 = Beamformer FPGA
            if (pNetMsg->m_head.m_src_sub_device_id == 2)
            {
                const CGemMk2BFStatusPacket* const pBF = (const CGemMk2BFStatusPacket* const)value;
                
                // Ignore status messages with no active connection
                if( !pBF->m_sonarAltIp ) return;

                status_msg.shutdown_status = pBF->m_shutdownStatus;
                status_msg.over_temperature_shutdown = (pBF->m_shutdownStatus & 0x0001);
                status_msg.out_of_water_shutdown     = (pBF->m_shutdownStatus & 0x0002);

                // Die Temp still requires the specific FPGA formula
                status_msg.bf_die_temp   = ((pBF->m_dieTemp * 503.975) / 1024.0) - 273.15;
                
                // Standard MK2 12-bit temperatures
                status_msg.bf_pcb_temp   = decode_mk2_temp(pBF->m_pcbTemp);
                status_msg.bf_comms_temp = decode_mk2_temp(pBF->m_commsTemp);
                status_msg.bf_tx_temp    = decode_mk2_temp(pBF->m_txTemp);
                status_msg.bf_psu_temp   = decode_mk2_temp(pBF->m_psuTemp);

                status_pub_->publish(status_msg);
            }
            // Device ID 4 = Data Acquisition FPGA 0
            else if (pNetMsg->m_head.m_src_sub_device_id == 4)
            {
                const CGemMk2DAStatusPacket* const pDA = (const CGemMk2DAStatusPacket* const)value;

                // Standard MK2 12-bit temperatures
                status_msg.da_pcb_temp      = decode_mk2_temp(pDA->m_pcbTemp);
                status_msg.da_afe0_top_temp = decode_mk2_temp(pDA->m_afe0TopTemp);
                status_msg.da_afe0_bot_temp = decode_mk2_temp(pDA->m_afe0BotTemp);
                status_msg.da_afe1_top_temp = decode_mk2_temp(pDA->m_afe1TopTemp);
                status_msg.da_afe1_bot_temp = decode_mk2_temp(pDA->m_afe1BotTemp);
                status_msg.da_afe2_top_temp = decode_mk2_temp(pDA->m_afe2TopTemp);
                status_msg.da_afe2_bot_temp = decode_mk2_temp(pDA->m_afe2BotTemp);
                status_msg.da_afe3_top_temp = decode_mk2_temp(pDA->m_afe3TopTemp);
                status_msg.da_afe3_bot_temp = decode_mk2_temp(pDA->m_afe3BotTemp);

                status_pub_->publish(status_msg);
            }
        }
        break;
        
        default:
            // Handle other messages if necessary
            break;
    }
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
    
    parametersCallback(initial_params);
}

// Intercepts live terminal changes and routes them to the Gemini API
rcl_interfaces::msg::SetParametersResult GeminiSonarNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters applied successfully";

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
            RangeFrequencyConfig freq_config;
            
            // Read from the incoming update if present, otherwise fall back to the node's current state
            int mode = (param.get_name() == "frequency_mode") ? param.as_int() : this->get_parameter("frequency_mode").as_int();
            double thresh = (param.get_name() == "range_threshold") ? param.as_double() : this->get_parameter("range_threshold").as_double();
            
            freq_config.m_frequency = static_cast<FREQUENCY>(mode);
            freq_config.m_rangeThreshold = thresh;
            
            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_RANGE_RESOLUTION, sizeof(RangeFrequencyConfig), &freq_config, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: 1200iK Frequency Mode Configured");
        }
        else if (param.get_name() == "ping_mode") {
            SequencerApi::SequencerPingMode ping_config;
            int p_mode = param.as_int();
            
            ping_config.m_bFreeRun = (p_mode == 0);
            ping_config.m_extTTLTrigger = (p_mode == 2);
            ping_config.m_manualTrigger = (p_mode == 3);
            if (p_mode == 1) ping_config.m_msInterval = 100; // 100ms Fixed interval 
            
            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_PING_MODE, sizeof(SequencerApi::SequencerPingMode), &ping_config, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: Ping Mode Configured");
        }
        else if (param.get_name() == "use_manual_sos" || param.get_name() == "manual_sos") {
            SequencerApi::SequencerSosConfig sos_config;
            
            sos_config.m_bUsedUserSos = (param.get_name() == "use_manual_sos") ? param.as_bool() : this->get_parameter("use_manual_sos").as_bool();
            sos_config.m_manualSos = (param.get_name() == "manual_sos") ? param.as_double() : this->get_parameter("manual_sos").as_double();
            
            SequencerApi::Svs5SetConfiguration(SequencerApi::SVS5_CONFIG_SOUND_VELOCITY, sizeof(SequencerApi::SequencerSosConfig), &sos_config, 0);
            RCLCPP_INFO(this->get_logger(), "Live Update: Speed of Sound Configured");
        }
    }
    return result;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeminiSonarNode>());
    rclcpp::shutdown();
    return 0;
}

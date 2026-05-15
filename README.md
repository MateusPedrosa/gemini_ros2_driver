
--------------------------------------------------------------------------------

# Tritech Gemini ROS 2 Driver (`gemini_ros2`)

ROS 2 driver for Tritech Gemini 1200iK multibeam imaging sonar, leveraging the Tritech `Svs5Sequencer` SDK to support both live hardware network streams and simulated data playback using proprietary Genesis Log Files (`.glf`).

This driver extracts high-resolution acoustic target imagery, detailed hardware telemetry (such as FPGA and TX temperatures), and parses auxiliary network sensors including GPS, Altimeter, Depth, and IMU data directly into standard ROS 2 `sensor_msgs`. It also allows the user to change the sonar settings such as range and frequency mode via ROS parameters.

This repository **does not** contain the Tritech Gemini SDK. To use this driver, you must independently download the SDK from Tritech and place it in your workspace.

---

## Installation & Docker Setup

This package is designed to run in an isolated Docker environment (ROS 2 Jazzy, Ubuntu 24.04) to ensure complete compatibility with the Tritech SDK and prevent dependency conflicts on your host machine.

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/gemini_ros2_driver.git
cd gemini_ros2_driver
```

## 2. Download and Place the Tritech SDK

1. Download the Gemini SDK from the Tritech customer portal (e.g., `GeminiSDK_v2.0.42.0_Ubuntu_24.04_arm-nvidia-nx` for ARM64 Jetsons).
2. Extract the SDK directly into the root of this cloned repository, in a directory named `GeminiSDK`.

Your directory structure should look like this:

    gemini_ros2_driver/
    ├── src/
    │   └── gemini_ros2/          # ROS 2 package
    ├── GeminiSDK/                # Extracted SDK
    ├── Dockerfile
    ├── docker-compose.yml
    └── README.md

## 3. Build the Docker Environment

The provided `Dockerfile` will automatically detect the SDK folder, install the necessary ROS 2 dependencies, copy the proprietary `.so` binaries into `/usr/local/lib`, and compile the `gemini_ros2` workspace.

```bash
docker compose build
```

---

## User Guide

### Running the Driver

To launch the ROS 2 node and the integrated Foxglove Bridge:

```bash
docker compose up -d
```

You can now open [Foxglove Studio](https://foxglove.dev), connect to `ws://localhost:8765`, and instantly visualize your sonar data!

### Live Hardware Mode vs. Simulation Mode

The driver is completely plug-and-play and can seamlessly switch between the physical hardware in the water and a recorded `.glf` log file on your desk. This is controlled by the `live_mode` parameter.

**To run with a recorded log file (Default):** The node will play the file specified in `log_file_path`. Note that acoustic parameters (Range, Gain) cannot alter recorded data.

```bash
ros2 launch gemini_ros2 gemini_sonar.launch.py live_mode:=False log_file_path:=/path/to/sample.glf
```

**To run with the live Sonar hardware:** Connect your PC/Jetson to the sonar's Ethernet network (default IP subnet 192.168.2.x) and launch the node in live mode.

```bash
ros2 launch gemini_ros2 gemini_sonar.launch.py live_mode:=True
```

### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/sonar/image` | `sensor_msgs/msg/Image` | 8-bit mono acoustic target imagery (polar sweep). |
| `/sonar/status` | `gemini_ros2/msg/SonarStatus` | Custom message containing MK2 Beamformer and Data Acquisition telemetry (Die Temps, PCB Temps, Safety Flags). |
| `/sonar/imu` | `sensor_msgs/msg/Imu` | Orientation (Quaternions) derived from `COMPASS_RECORD`. |
| `/sonar/gps` | `sensor_msgs/msg/NavSatFix` | Coordinates derived from `GPS_RECORD`. |
| `/sonar/altitude` | `std_msgs/msg/Float64` | Altitude in meters derived from `ALTIMETER_RECORD`. |
| `/sonar/depth` | `std_msgs/msg/Float64` | Depth in meters derived from `BATHY_RECORD`. |

---

## Dynamic Configuration (ROS 2 Parameters)

This node utilizes `rclcpp::ParameterEventHandler`, meaning you can adjust the sonar's hardware settings on the fly directly from the terminal without restarting the node.

**Example:**

```bash
ros2 param set /gemini_sonar_node range 45.0
ros2 param set /gemini_sonar_node gain 75
```

### Supported Parameters

#### Environment & Setup

- `live_mode` (bool): `True` connects to physical network port. `False` plays back a log file.
- `log_file_path` (string): Absolute path to the `.glf` file for playback.

#### Acoustic Controls (Live Mode Only)

- `range` (double): Maximum acoustic range in meters.
- `gain` (int): Receiver gain percentage (0–100).
- `ping_mode` (int): `0` = Free Run, `1` = Fixed Interval, `2` = Ext TTL, `3` = Manual.
- `manual_ping` (bool): If true, fires a single ping (Requires `ping_mode` = 3).
- `use_manual_sos` (bool): Override the sonar's internal velocimeter.
- `manual_sos` (double): Speed of sound in water (m/s).

#### Gemini 1200iK MK2 Specific

- `frequency_mode` (int): `0` = Auto, `1` = Low (720kHz), `2` = High (1.2MHz).
- `range_threshold` (double): Range limit (meters) where Auto-Frequency swaps from High to Low.
- `high_resolution` (bool): Doubles the range lines for finer detail.
- `chirp_mode` (int): `0` = Disabled, `1` = Enabled, `2` = Auto.

#### Advanced System / Network Settings

- `cpu_performance` (int): Throttles processing. `0` = Low, `1` = Medium, `2` = High, `3` = Ultra High.
- `h264_compression` (bool): Compresses the network stream to save bandwidth.
- `aux_baud_rate` (int): Baud rate for physical AUX port (e.g., 115200).
- `aux_rs232` (bool): `True` = RS232, `False` = RS485.

***
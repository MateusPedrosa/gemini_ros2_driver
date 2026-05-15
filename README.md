
--------------------------------------------------------------------------------

# Tritech Gemini ROS 2 Driver (`gemini_ros2`)

ROS 2 driver for the Tritech Gemini 1200iK multibeam imaging sonar. Uses the Tritech `Svs5Sequencer` SDK to support both live hardware streams and simulated playback from Genesis Log Files (`.glf`).

The driver publishes acoustic images, hardware telemetry, IMU orientation, raw IMU sensor data, magnetometer, and GPS into standard ROS 2 message types. All acoustic settings can be changed live at runtime via ROS 2 parameters without restarting the node.

> **This repository does not include the Tritech Gemini SDK.** You must obtain it independently from the Tritech customer portal and place it in the workspace as described below.

---

## Directory Structure

```
tritechsdk/
├── src/
│   └── gemini_ros2/          # ROS 2 package (this driver)
├── GeminiSDK/                # Tritech SDK — place here after download
│   ├── include/
│   └── libs/
├── Dockerfile
├── docker-compose.yml
└── README.md
```

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/yourusername/gemini_ros2_driver.git
cd gemini_ros2_driver
```

### 2. Download and place the Tritech SDK

Download the SDK from the Tritech customer portal. You want the build matching your target platform — for NVIDIA Jetson (ARM64):

```
GeminiSDK_vX.X.X_Ubuntu_24.04_arm-nvidia-nx
```

Extract the archive and rename or move the resulting folder to `GeminiSDK/` at the root of this repository (alongside `src/`).

### 3. Obtain a sample GLF recording (optional, for simulation)

A set of real `.glf` recordings is available at:

```
https://github.com/OniDaito/pytritech_testdata
```

Clone or download that repository and note the path to a `.glf` file. You will pass it to the driver via the `log_file_path` parameter.

### 4. Build the Docker environment

```bash
docker compose build
```

The `Dockerfile` installs ROS 2 dependencies, copies the SDK libraries into `/usr/local/lib`, and compiles the `gemini_ros2` package inside the container.

---

## Running the Driver

Start the container and the Foxglove Bridge:

```bash
docker compose up -d
```

Then open [Foxglove Studio](https://foxglove.dev) and connect to `ws://localhost:8765` to visualise the sonar data.

To launch the node directly:

```bash
ros2 launch gemini_ros2 gemini_sonar.launch.py
```

---

## Simulation Mode (GLF Playback)

Set `live_mode` to `False` (overriding the default) and point `log_file_path` at a `.glf` file. The driver will replay the recording at real-time speed by default.

```bash
ros2 launch gemini_ros2 gemini_sonar.launch.py \
  live_mode:=False \
  log_file_path:=/path/to/recording.glf
```

> **Note:** Acoustic parameters (range, gain, etc.) are sent to the SDK during playback but have no effect on the pre-recorded data. They will take effect when you switch to live hardware.

Loop the recording and switch to free-run speed:

```bash
ros2 param set /gemini_sonar_node loop_playback true
ros2 param set /gemini_sonar_node playback_speed 0
```

Pause and resume:

```bash
ros2 service call /sonar/playback/pause std_srvs/srv/SetBool "{data: true}"
ros2 service call /sonar/playback/pause std_srvs/srv/SetBool "{data: false}"
```

Stop playback:

```bash
ros2 service call /sonar/playback/stop std_srvs/srv/Trigger "{}"
```

---

## Live Hardware Mode

### Connecting to the Sonar

The Gemini 1200iK communicates over Ethernet. The sonar's default IP address is **`192.168.2.201`** on subnet `255.255.255.0`.

**Step 1 — Wire up**

Connect your host machine (or Jetson) to the sonar's Ethernet port directly or through a network switch. The sonar powers over its cable, so no separate power connection is needed beyond the tether.

**Step 2 — Configure your host network interface**

Your host's Ethernet interface must be on the same `192.168.2.x` subnet. For example, set it to a static address of `192.168.2.100`:

```bash
# Find your Ethernet interface name
ip link show

# Set a static address (replace eth0 with your interface)
sudo ip addr add 192.168.2.100/24 dev eth0
sudo ip link set eth0 up
```

Or use your OS network manager to assign a static IP of `192.168.2.100`, netmask `255.255.255.0`, on the wired interface connected to the sonar.

**Step 3 — Verify the sonar is reachable**

```bash
ping 192.168.2.201
```

You should see replies. If not, check the cable and confirm your interface address is in the same subnet.

**Step 4 — Launch in live mode**

```bash
ros2 launch gemini_ros2 gemini_sonar.launch.py
```

The SDK auto-discovers the sonar via broadcast on the subnet — no IP address needs to be passed to the driver. You should immediately see `sonar/image` frames and `sonar/status` messages being published.

---

## Recording to GLF

To record a new `.glf` log file from a live session, set a target directory and call the record service:

```bash
# Set where the GLF file will be written (at launch time via parameter)
ros2 launch gemini_ros2 gemini_sonar.launch.py record_path:=/data/recordings

# Start and stop recording at any time
ros2 service call /sonar/record std_srvs/srv/SetBool "{data: true}"
ros2 service call /sonar/record std_srvs/srv/SetBool "{data: false}"
```

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/sonar/image` | `sensor_msgs/Image` | 8-bit mono acoustic target image (polar sweep, `mono8` encoding) |
| `/sonar/status` | `gemini_ros2/SonarStatus` | Hardware telemetry: temperatures (die, PCB, TX, PSU, AFE), shutdown flags |
| `/sonar/imu` | `sensor_msgs/Imu` | Orientation quaternion from compass/AHRS heading-pitch-roll |
| `/sonar/imu_raw` | `sensor_msgs/Imu` | Raw gyroscope (rad/s) and accelerometer (m/s²) from AHRS — live mode and recordings that include a connected AHRS unit |
| `/sonar/mag` | `sensor_msgs/MagneticField` | Magnetometer X/Y/Z — same availability as `imu_raw`; units are normalised (not Tesla) |
| `/sonar/gps` | `sensor_msgs/NavSatFix` | Latitude, longitude, altitude from a connected GPS source |

> `sonar/imu_raw`, `sonar/mag`, and `sonar/gps` are only populated if the corresponding hardware (AHRS unit, GPS receiver) was connected and logging was enabled when the `.glf` was recorded, or when running live with that hardware present.

---

## Services

| Service | Type | Description |
|---|---|---|
| `/sonar/playback/pause` | `std_srvs/SetBool` | `true` = pause, `false` = resume playback |
| `/sonar/playback/stop` | `std_srvs/Trigger` | Stop and rewind to beginning |
| `/sonar/record` | `std_srvs/SetBool` | `true` = start recording to GLF, `false` = stop |

---

## ROS 2 Parameters

All parameters can be set at launch time or changed live with `ros2 param set` without restarting the node.

### Environment

| Parameter | Type | Default | Description |
|---|---|---|---|
| `live_mode` | `bool` | `true` | `true` = connect to physical sonar, `false` = play back a GLF file |
| `log_file_path` | `string` | — | Absolute path to `.glf` file (playback mode only) |
| `record_path` | `string` | `""` | Directory for GLF recording output; uses SDK default if empty |

### Acoustic Controls

| Parameter | Type | Default | Description |
|---|---|---|---|
| `range` | `double` | `20.0` | Maximum range in metres (1–120) |
| `gain` | `int` | `50` | Receiver gain percentage (1–100) |
| `ping_mode` | `int` | `0` | `0` Free-run, `1` Fixed interval (100 ms), `2` Ext TTL trigger, `3` Manual |
| `use_manual_sos` | `bool` | `false` | Override the sonar's internal velocimeter |
| `manual_sos` | `double` | `1500.0` | Speed of sound in m/s (used when `use_manual_sos` is true) |

### Gemini 1200iK Specific

| Parameter | Type | Default | Description |
|---|---|---|---|
| `frequency_mode` | `int` | `0` | `0` Auto, `1` Low (720 kHz), `2` High (1.2 MHz), `3` Combined (interleaved) |
| `range_threshold` | `double` | `20.0` | Range in metres at which Auto frequency mode switches from High to Low |
| `high_resolution` | `bool` | `true` | Doubles range line count for finer detail |
| `chirp_mode` | `int` | `2` | `0` Disabled, `1` Enabled, `2` Auto |

### Image & Hardware

| Parameter | Type | Default | Description |
|---|---|---|---|
| `sonar_inverted` | `bool` | `false` | Set `true` if the sonar is mounted upside-down |
| `aperture` | `double` | `120.0` | Field of view in degrees — `120.0` or `65.0` |
| `image_quality` | `int` | `3` | SDK processing load: `0` Low (256 beams), `1` Medium, `2` High (512 beams), `3` Ultra (512–1024 beams) |

### Playback

| Parameter | Type | Default | Description |
|---|---|---|---|
| `loop_playback` | `bool` | `false` | Repeat the GLF file when it ends |
| `playback_speed` | `int` | `1` | `0` Free-run (as fast as possible), `1` Real-time |

---

## Quick Reference

```bash
# Change range live
ros2 param set /gemini_sonar_node range 45.0

# Switch to high-frequency only
ros2 param set /gemini_sonar_node frequency_mode 2

# Reduce FOV for a narrower, higher-resolution sector
ros2 param set /gemini_sonar_node aperture 65.0

# Invert image for upside-down mounting
ros2 param set /gemini_sonar_node sonar_inverted true

# Check all current parameter values
ros2 param list /gemini_sonar_node
ros2 param dump /gemini_sonar_node
```

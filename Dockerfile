FROM --platform=linux/arm64 ros:jazzy-ros-base

# 1. Install ROS 2 packages AND Tritech SDK GUI dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-foxglove-bridge \
    qtbase5-dev \
    libfreetype6-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. Copy the Tritech SDK into the image and install it permanently
COPY GeminiSDK /workspace/GeminiSDK
WORKDIR /workspace/GeminiSDK
RUN chmod +x InstallSDK.sh && ./InstallSDK.sh && ldconfig

# 3. Configure the ROS 2 environment variables permanently
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/build/install/setup.bash" >> ~/.bashrc

# 4. Set the default directory when you enter the container
WORKDIR /workspace/build
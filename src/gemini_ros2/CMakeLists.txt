cmake_minimum_required(VERSION 3.16)
project(gemini_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SonarStatus.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Include the Gemini SDK headers
include_directories(
  /workspace/GeminiSDK/include
)

add_executable(gemini_sonar_node src/gemini_sonar_node.cpp)
target_include_directories(gemini_sonar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(gemini_sonar_node rclcpp sensor_msgs std_msgs std_srvs tf2)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# Link the Tritech ARM64 libraries and generated message interfaces
target_link_libraries(gemini_sonar_node
  "${cpp_typesupport_target}"
  /workspace/GeminiSDK/libs/Svs5SeqLib/Linux/aarch64/Release/libSvs5SeqLib.so
  /workspace/GeminiSDK/libs/GenesisSerializer/Linux/aarch64/Release/libGenesisSerializer.so
  /workspace/GeminiSDK/libs/GeminiComms/Linux/aarch64/Release/libGeminiComms.so
  /workspace/GeminiSDK/libs/DataProcessor/Linux/aarch64/Release/libDataProcessor.so
  pthread
)

install(TARGETS gemini_sonar_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
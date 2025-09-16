cmake_minimum_required(VERSION 3.12)
project(upper_body_demo LANGUAGES CXX)

# Enforce C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 and its message API
find_package(ament_cmake REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(sensor_msgs REQUIRED)

# Other deps
find_package(Boost         REQUIRED COMPONENTS program_options)
find_package(spdlog        REQUIRED)
find_package(Eigen3)
find_package(
  ethercat-cpp
  REQUIRED
  PATHS 
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/cmake"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../"
  NO_DEFAULT_PATH
)
######################################################
# Wave hand demo executable ##########################
add_executable(wave_hand_demo
  src/wavehand/main.cpp
  src/power.cpp
  src/wavehand/WaveHandDemo.cpp
)

target_link_libraries(wave_hand_demo
  rclcpp::rclcpp
  Boost::program_options
  spdlog::spdlog
  ethercat-cpp::ethercat-cpp
)

ament_target_dependencies(wave_hand_demo
  ethercat-cpp
  rclcpp
  std_msgs
  Boost
  spdlog
)

target_include_directories(wave_hand_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
######################################################


######################################################
# teleop executable ##################################
add_executable(teleop
  src/teleop/main.cpp
  src/power.cpp
  src/teleop/armPosiSender.cpp
  src/wrist_IK_FK.cpp
  src/udp_tool.cpp
)

# Link in ROS 2 libraries (plain signature)
target_link_libraries(teleop
  rclcpp::rclcpp
  Boost::program_options
  spdlog::spdlog
  ethercat-cpp::ethercat-cpp
  Eigen3::Eigen
)

ament_target_dependencies(teleop
  ethercat-cpp
  rclcpp
  sensor_msgs
  std_msgs
  Boost
  spdlog
)

target_include_directories(teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
######################################################

######################################################
# enable all joints ##################################
add_executable(enable
  src/enable/main.cpp
)

# Link in ROS 2 libraries (plain signature)
target_link_libraries(enable
  rclcpp::rclcpp
  Boost::program_options
)

ament_target_dependencies(enable
  rclcpp
  std_msgs
  Boost
)

target_include_directories(enable PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
######################################################

# # RPATH so the custom ethercat lib is found
# set_target_properties(wave_hand_demo PROPERTIES
#   INSTALL_RPATH "$ORIGIN/../lib"
#   # BUILD_WITH_INSTALL_RPATH TRUE
# )

# Install rules
install(TARGETS 
  wave_hand_demo
  teleop
  enable
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/src/upper_body_demo/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "libethercat-cpp.so*"
)

ament_package()

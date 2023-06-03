################################################################################
################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(rplidar_ros2)
 
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()
 
set(RPLIDAR_SDK_PATH "./sdk/")
 
FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/win32/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)
 
################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
################################################################################
# Build
################################################################################
include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
)
 
 
add_executable(rplidar_scan_subscriber src/rplidar_scan_subscriber.cpp ${RPLIDAR_SDK_SRC})
 
 
ament_target_dependencies(rplidar_scan_subscriber
  rclcpp
  std_srvs
  sensor_msgs
  OpenCV
  geometry_msgs
)
 
################################################################################
# Install
################################################################################
 
 
install(
  TARGETS rplidar_scan_subscriber
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
 
################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_package()

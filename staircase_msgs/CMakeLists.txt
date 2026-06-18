cmake_minimum_required(VERSION 3.5) # Recommended for ROS 2
project(staircase_msgs)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "Defaulting CMAKE_BUILD_TYPE to ${CMAKE_BUILD_TYPE}")
endif()

# Suppress CMake developer warnings (equivalent to -Wno-dev on command line)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS TRUE)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StaircaseMeasurement.msg"
  "msg/StaircaseMsg.msg"
  DEPENDENCIES
   geometry_msgs
   std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY msg
    DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()
cmake_minimum_required(VERSION 3.5) # Recommended for ROS 2
project(staircase_perception)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "Defaulting CMAKE_BUILD_TYPE to ${CMAKE_BUILD_TYPE}")
endif()

# Suppress CMake developer warnings (equivalent to -Wno-dev on command line)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS TRUE)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(staircase_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

# Define source file groups (optional, but good for organization)
file(GLOB line_extraction_src
  "src/utils/line_extraction/line.cpp"
  "src/utils/line_extraction/line_extractor.cpp"
)

file(GLOB single_robot_pipeline_src
  "src/core/stair_detector.cpp"
  "src/core/multi_stair_manager.cpp"
  "src/core/staircase_model.cpp"
)

# Executable for staircase_estimation_robot_node
add_executable(staircase_estimation_robot_node "src/ros2/staircase_estimation_robot_node.cpp" ${single_robot_pipeline_src} ${line_extraction_src})

target_include_directories(staircase_estimation_robot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(staircase_estimation_robot_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
  staircase_msgs
)

target_link_libraries(staircase_estimation_robot_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen 
)

# Install executables
install(TARGETS
  staircase_estimation_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers 
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config
# install(DIRECTORY rviz
#     DESTINATION share/${PROJECT_NAME}/rviz
# )

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/staircase_marker_publisher.py
  scripts/pointcloud_registration_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
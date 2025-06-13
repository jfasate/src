cmake_minimum_required(VERSION 3.8)
project(rallycar_scan_matching)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Create the scan matching node executable
add_executable(scanmatch_node 
  src/scanmatch_node.cpp 
  src/transform.cpp 
  src/correspond.cpp 
  src/visualization.cpp
)

# Link the node with dependencies
ament_target_dependencies(scanmatch_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
)

# Install executables
install(TARGETS
  scanmatch_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)


# Export dependencies
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  geometry_msgs 
  visualization_msgs 
  tf2_ros
)

ament_package()

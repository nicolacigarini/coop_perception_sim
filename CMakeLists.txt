cmake_minimum_required(VERSION 3.8)
project(coop_perception_sim) #cooperative perception simulation
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(odom_map_remapper src/odom_map_remapper.cpp)
ament_target_dependencies(odom_map_remapper rclcpp nav_msgs geometry_msgs tf2_msgs)

add_executable(multi_lidar_slam src/multi_lidar_slam.cpp)
ament_target_dependencies(multi_lidar_slam rclcpp nav_msgs geometry_msgs tf2 sensor_msgs)

install(TARGETS
  odom_map_remapper
  multi_lidar_slam
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
launch
config
rviz
maps
DESTINATION share/${PROJECT_NAME})

ament_package()
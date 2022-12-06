cmake_minimum_required(VERSION 3.8)
project(tb3_obstacle_avoidance_pkg)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(walker_algorithm src/walker_algorithm.cpp)
ament_target_dependencies(walker_algorithm rclcpp geometry_msgs sensor_msgs)

install(TARGETS walker_algorithm
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()

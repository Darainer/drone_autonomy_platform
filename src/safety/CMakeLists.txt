cmake_minimum_required(VERSION 3.8)
project(safety)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_autonomy_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(safety_node rclcpp drone_autonomy_msgs mavros_msgs)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

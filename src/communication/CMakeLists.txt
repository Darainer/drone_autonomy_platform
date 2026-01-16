cmake_minimum_required(VERSION 3.8)
project(communication)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(drone_autonomy_msgs REQUIRED)

add_executable(communication_node src/communication_node.cpp)
ament_target_dependencies(communication_node rclcpp mavros_msgs drone_autonomy_msgs)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

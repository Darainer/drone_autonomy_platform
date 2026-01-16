cmake_minimum_required(VERSION 3.8)
project(autonomy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_autonomy_msgs REQUIRED)

add_executable(autonomy_node src/autonomy_node.cpp)
ament_target_dependencies(autonomy_node rclcpp drone_autonomy_msgs)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

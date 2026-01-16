cmake_minimum_required(VERSION 3.8)
project(perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(isaac_ros_visual_slam_interfaces REQUIRED)
find_package(isaac_ros_dnn_inference REQUIRED)
find_package(drone_autonomy_msgs REQUIRED)

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node rclcpp isaac_ros_visual_slam_interfaces isaac_ros_dnn_inference drone_autonomy_msgs)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

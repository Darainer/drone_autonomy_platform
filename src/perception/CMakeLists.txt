cmake_minimum_required(VERSION 3.8)
project(perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(drone_autonomy_msgs REQUIRED)

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node
    rclcpp sensor_msgs vision_msgs drone_autonomy_msgs)

install(TARGETS perception_node
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    src/detection_visualizer.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME detection_visualizer
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

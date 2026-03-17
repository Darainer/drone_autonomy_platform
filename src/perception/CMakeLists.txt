cmake_minimum_required(VERSION 3.8)
project(perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(drone_autonomy_msgs REQUIRED)

# C++ perception fusion node
add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node
    rclcpp sensor_msgs vision_msgs drone_autonomy_msgs)

install(TARGETS perception_node
    DESTINATION lib/${PROJECT_NAME}
)

# Python RF-DETR-S inference node
install(PROGRAMS src/rfdetr_node.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME rfdetr_node
)

install(PROGRAMS
    src/detection_visualizer.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME detection_visualizer
)

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

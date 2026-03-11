cmake_minimum_required(VERSION 3.8)
project(mock_fcu)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mock_fcu_node src/mock_fcu_node.cpp)
ament_target_dependencies(mock_fcu_node rclcpp mavros_msgs sensor_msgs)

install(TARGETS mock_fcu_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(safety)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_autonomy_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(safety_node rclcpp drone_autonomy_msgs mavros_msgs)

add_executable(battery_monitor src/battery_monitor.cpp)
ament_target_dependencies(battery_monitor rclcpp sensor_msgs mavros_msgs)

install(TARGETS
    safety_node
    battery_monitor
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_battery_monitor
        test/test_battery_monitor.cpp
    )
    ament_target_dependencies(test_battery_monitor
        rclcpp sensor_msgs mavros_msgs
    )

    ament_add_gtest(test_safety_node
        test/test_safety_node.cpp
    )
    ament_target_dependencies(test_safety_node
        rclcpp mavros_msgs
    )
endif()

ament_package()

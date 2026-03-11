cmake_minimum_required(VERSION 3.8)
project(control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_autonomy_msgs REQUIRED)

add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp drone_autonomy_msgs)

install(TARGETS control_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_control_node
        test/test_control_node.cpp
    )
    ament_target_dependencies(test_control_node
        rclcpp drone_autonomy_msgs
    )
endif()

ament_package()

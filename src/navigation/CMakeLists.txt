cmake_minimum_required(VERSION 3.8)
project(navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(drone_autonomy_msgs REQUIRED)

add_executable(navigation_node src/navigation_node.cpp)
ament_target_dependencies(navigation_node rclcpp nav2_core drone_autonomy_msgs)

install(TARGETS navigation_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_navigation_node
        test/test_navigation_node.cpp
    )
    ament_target_dependencies(test_navigation_node
        rclcpp drone_autonomy_msgs
    )
endif()

ament_package()

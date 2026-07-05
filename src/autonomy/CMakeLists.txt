cmake_minimum_required(VERSION 3.8)
project(autonomy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_autonomy_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Node class + pure survey-validation logic (DES-003), shared between the
# autonomy_node executable and the test binary below.
add_library(autonomy_core
    src/autonomy_node.cpp
    src/survey_validation.cpp
)
target_include_directories(autonomy_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(autonomy_core rclcpp drone_autonomy_msgs geometry_msgs)

add_executable(autonomy_node src/main.cpp)
target_link_libraries(autonomy_node autonomy_core)
ament_target_dependencies(autonomy_node rclcpp drone_autonomy_msgs)

install(TARGETS autonomy_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # TP-002 TS-01/TS-02 — ROS-graph tests for survey dispatch/rejection.
    # Verifies: MAP-6
    ament_add_gtest(test_survey_dispatch test/test_survey_dispatch.cpp)
    target_include_directories(test_survey_dispatch PRIVATE include)
    target_link_libraries(test_survey_dispatch autonomy_core)
    ament_target_dependencies(test_survey_dispatch rclcpp drone_autonomy_msgs geometry_msgs)
endif()

ament_package()

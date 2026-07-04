cmake_minimum_required(VERSION 3.8)
project(navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(drone_autonomy_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Pure-geometry survey coverage-trajectory generator (DES-003 "navigation_node
# behavior") — no ROS dependencies, unit-tested directly by TS-03.
add_library(survey_planner src/survey_planner.cpp)
target_include_directories(survey_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(navigation_node src/navigation_node.cpp)
target_link_libraries(navigation_node survey_planner)
ament_target_dependencies(navigation_node rclcpp nav2_core drone_autonomy_msgs geometry_msgs)

install(TARGETS navigation_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS survey_planner
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # TP-002 TS-03 — SurveyPlanner coverage geometry properties (Verifies: MAP-1).
    ament_add_gtest(survey_planner_test tests/survey_planner_test.cpp)
    target_link_libraries(survey_planner_test survey_planner)
endif()

ament_package()

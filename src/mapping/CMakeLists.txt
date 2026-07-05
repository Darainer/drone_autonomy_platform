cmake_minimum_required(VERSION 3.8)
project(mapping)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(drone_autonomy_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Pure-logic dataset writer (DES-004 "Dataset format": manifest.yaml,
# poses.csv, sha256 checksums) — no ROS dependencies, unit-tested directly
# by TS-07 (format schema + offload verifiability).
add_library(dataset_writer src/dataset_writer.cpp)
target_include_directories(dataset_writer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(survey_recorder_node src/survey_recorder_node.cpp)
target_link_libraries(survey_recorder_node dataset_writer ${OpenCV_LIBS})
target_include_directories(survey_recorder_node PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(survey_recorder_node
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    message_filters
    drone_autonomy_msgs
    cv_bridge
)

install(TARGETS survey_recorder_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS dataset_writer
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # TP-002 TS-07 — dataset format schema + offload verifiability (Verifies: MAP-3).
    ament_add_gtest(dataset_writer_test tests/dataset_writer_test.cpp)
    target_link_libraries(dataset_writer_test dataset_writer)

    # TP-002 TS-05/TS-06 — sync budget, recording window (Verifies: MAP-2).
    # ROS-graph replay test; requires the F3 fixture bag
    # (tests/fixtures/f3_survey_replay/, CAP-001 WP-2 T2.3) and self-skips
    # until that fixture is checked in.
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(tests/test_survey_recorder_replay.py TIMEOUT 300)
endif()

ament_package()

# CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(drone_autonomy_platform VERSION 0.1.0 LANGUAGES CXX)

# ============================================================================
# Build Options
# ============================================================================
option(BUILD_TESTING "Build tests" ON)
option(BUILD_SIMULATION "Build simulation components" ON)
option(BUILD_HARDWARE "Build hardware drivers" OFF)
option(ENABLE_SAFETY_CHECKS "Enable runtime safety checks" ON)
option(ENABLE_SANITIZERS "Enable address/thread sanitizers" OFF)
option(ENABLE_COVERAGE "Enable code coverage" OFF)

# ============================================================================
# Compiler Configuration
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Safety-critical: Treat warnings as errors in CI
if(DEFINED ENV{CI})
    add_compile_options(-Werror)
endif()

add_compile_options(
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter
    -Wcast-align
    -Wformat=2
    -Wmissing-declarations
)

# Real-time considerations
if(ENABLE_SAFETY_CHECKS)
    add_compile_definitions(SAFETY_CHECKS_ENABLED)
endif()

# Sanitizers for development
if(ENABLE_SANITIZERS)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

# Coverage
if(ENABLE_COVERAGE)
    add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
    add_link_options(--coverage)
endif()

# ============================================================================
# Dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

# Optional dependencies
if(BUILD_SIMULATION)
    find_package(gazebo_ros QUIET)
endif()

# ============================================================================
# Subdirectories
# ============================================================================
add_subdirectory(src/common)
add_subdirectory(src/safety)
add_subdirectory(src/perception)
add_subdirectory(src/navigation)
add_subdirectory(src/control)
add_subdirectory(src/autonomy)
add_subdirectory(src/communication)

if(BUILD_SIMULATION)
    # add_subdirectory(src/simulation)
endif()

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_pytest REQUIRED)
    
    ament_lint_auto_find_test_dependencies()
    # add_subdirectory(test)
endif()

# ============================================================================
# Installation
# ============================================================================
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
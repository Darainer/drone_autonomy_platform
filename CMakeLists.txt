cmake_minimum_required(VERSION 3.22)
project(drone_autonomy_platform VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTING "Build tests" ON)
option(BUILD_SIMULATION "Build simulation" ON)
option(ENABLE_SAFETY_CHECKS "Enable safety checks" ON)

if(ENABLE_SAFETY_CHECKS)
    add_compile_definitions(SAFETY_CHECKS_ENABLED)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

add_subdirectory(src/common)
add_subdirectory(src/safety)
add_subdirectory(src/perception)
add_subdirectory(src/navigation)
add_subdirectory(src/control)
add_subdirectory(src/autonomy)
add_subdirectory(src/communication)
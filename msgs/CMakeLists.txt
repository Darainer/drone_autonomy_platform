cmake_minimum_required(VERSION 3.8)
project(drone_autonomy_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "ros2/AttitudeCommand.msg"
  "ros2/Mission.msg"
  "ros2/SensorData.msg"
  "ros2/Trajectory.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs nav_msgs
)

ament_package()

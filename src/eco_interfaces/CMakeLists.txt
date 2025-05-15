cmake_minimum_required(VERSION 3.8)
project(eco_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneDetection.msg"
  "msg/Obstacle.msg"
  "msg/Obstacles.msg"
  "msg/ParkingArea.msg"
  "msg/StopSign.msg"
  "msg/SystemState.msg"
  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
)

ament_package()

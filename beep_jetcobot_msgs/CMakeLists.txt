cmake_minimum_required(VERSION 3.8)
project(beep_jetcobot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlace.action"
)

ament_package()

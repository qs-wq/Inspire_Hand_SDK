cmake_minimum_required(VERSION 3.8)
project(eg5cd1_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperState.msg"
  "msg/SetInt32.msg"
  "srv/TriggerForHand.srv"
  "srv/SetInt32Value.srv"
  "srv/GetScalarForHand.srv"
  "srv/ForceModeGrasp.srv"
  "srv/ForceModeOpen.srv"
  "srv/TouchModeGrasp.srv"
  "srv/TouchModeOpen.srv"
  DEPENDENCIES std_msgs
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(rh524j1_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Setangle.srv"
  "srv/Getangleact.srv"
  "srv/Getangleset.srv"
  "srv/Setpos.srv"
  "srv/Getposact.srv"
  "srv/Getposset.srv"
  "srv/Setspeed.srv"
  "srv/Getspeedset.srv"
  "srv/Setforce.srv"
  "srv/Getforceact.srv"
  "srv/Getforceset.srv"
  "srv/Getcurrentact.srv"
  "srv/Geterror.srv"
  "srv/Gettemp.srv"
  "srv/Setgestureno.srv"
  "srv/Setmode.srv"
  "srv/Setid.srv"
  "srv/Setbaudrate.srv"
  "srv/Setclearerror.srv"
  "srv/Setresetpara.srv"
  "srv/Setgestureforceclb.srv"
  "srv/Setdefaultspeed.srv"
  "srv/Setdefaultforceset.srv"
  "srv/Setpause.srv"
  "srv/Setstop.srv"
  "srv/Getstatus.srv"

  "msg/GetAngleAct1.msg"
  "msg/GetForceAct1.msg"
  "msg/GetCurrentAct1.msg"

  "msg/SetAngle1.msg"
  "msg/SetForce1.msg"
  "msg/SetCurrent1.msg"
  "msg/SetSpeed1.msg"

  DEPENDENCIES  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

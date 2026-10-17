cmake_minimum_required(VERSION 3.8)
project(kinematics_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

function(ensure_interface_file relative_path content)
  set(target "${CMAKE_CURRENT_SOURCE_DIR}/${relative_path}")
  if(NOT EXISTS "${target}")
    get_filename_component(target_dir "${target}" DIRECTORY)
    file(MAKE_DIRECTORY "${target_dir}")
    file(WRITE "${target}" "${content}")
    message(WARNING "Created missing kinematics_msgs interface file: ${relative_path}")
  endif()
endfunction()

ensure_interface_file("msg/JointRange.msg" [=[float64 min
float64 max
]=])
ensure_interface_file("msg/JointsRange.msg" [=[kinematics_msgs/JointRange joint1
kinematics_msgs/JointRange joint2
kinematics_msgs/JointRange joint3
kinematics_msgs/JointRange joint4
kinematics_msgs/JointRange joint5
]=])
ensure_interface_file("msg/Link.msg" [=[float64 base_link
float64 link1
float64 link2
float64 link3
float64 end_effector_link
]=])
ensure_interface_file("srv/GetJointRange.srv" [=[
---
bool success
kinematics_msgs/JointsRange data
]=])
ensure_interface_file("srv/GetLink.srv" [=[
---
bool success
kinematics_msgs/Link data
]=])
ensure_interface_file("srv/GetRobotPose.srv" [=[
---
bool success
bool solution
geometry_msgs/Pose pose
]=])
ensure_interface_file("srv/SetJointRange.srv" [=[kinematics_msgs/JointsRange data
---
bool success
string message
]=])
ensure_interface_file("srv/SetJointValue.srv" [=[float32[] joint_value
---
bool success
bool solution
geometry_msgs/Pose pose
]=])
ensure_interface_file("srv/SetLink.srv" [=[kinematics_msgs/Link data
---
bool success
string message
]=])
ensure_interface_file("srv/SetRobotPose.srv" [=[float64[] position
float64 pitch
float64[] pitch_range
float64 resolution
---
bool success
uint16[] pulse
uint16[] current_pulse
float64[] rpy
float64 min_variation
]=])

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/JointRange.msg"
    "msg/JointsRange.msg"
    "msg/Link.msg"
    "srv/GetJointRange.srv"
    "srv/GetLink.srv"
    "srv/GetRobotPose.srv"
    "srv/SetJointRange.srv"
    "srv/SetJointValue.srv"
    "srv/SetLink.srv"
    "srv/SetRobotPose.srv"
    DEPENDENCIES geometry_msgs
 )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

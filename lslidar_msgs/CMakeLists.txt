cmake_minimum_required(VERSION 3.5)
project(lslidar_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED) 

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LslidarInformation.msg"
  "msg/LslidarPacket.msg"
  "srv/AngleDistortionCorrection.srv"
  "srv/FrameRate.srv"
  "srv/InvalidData.srv"
  "srv/IpAndPort.srv"
  "srv/MotorControl.srv"
  "srv/MotorSpeed.srv"
  "srv/PowerControl.srv"
  "srv/RfdRemoval.srv"
  "srv/StandbyMode.srv"
  "srv/TailRemoval.srv"
  "srv/TimeMode.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
ament_package()

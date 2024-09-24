cmake_minimum_required(VERSION 3.5)
project(lslidar_ls_driver)

set(CMAKE_BUILD_TYPE Release)
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(libpcap_LIBRARIES -lpcap)

find_package(Boost REQUIRED COMPONENTS)
find_package(angles REQUIRED)
find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# 禁用Python接口生成
set(ROSIDL_GENERATE_PYTHON OFF CACHE BOOL "" FORCE)

# 指定消息和服务文件
set(msg_files
    "msg/LslidarLsPacket.msg"
    # etc
)

set(srv_files
    "srv/AngleDistortionCorrection.srv"
    "srv/DataIp.srv"
    "srv/DevPort.srv"
    "srv/DestinationIp.srv"
    "srv/DataPort.srv"
    "srv/FrameRate.srv"
    "srv/InvalidData.srv"
    "srv/StandbyMode.srv"
    "srv/TimeService.srv"
    # etc
)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_generators rosidl_default_runtime)
ament_export_include_directories(include)

# 添加可执行文件
add_executable(lslidar_ls_driver_node src/lslidar_driver_node.cpp src/lslidar_driver.cpp src/input.cpp)

# 确保消息和服务接口在可执行文件之前生成
add_dependencies(lslidar_ls_driver_node lslidar_ls_driver__rosidl_typesupport_cpp)

# 目标接口
rosidl_target_interfaces(lslidar_ls_driver_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 链接库
target_link_libraries(lslidar_ls_driver_node ${rclcpp_LIBRARIES} ${libpcap_LIBRARIES} ${Boost_LIBRARIES})
ament_target_dependencies(lslidar_ls_driver_node rclcpp std_msgs sensor_msgs pcl_conversions)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch params rviz_cfg
    DESTINATION share/${PROJECT_NAME})

install(TARGETS
    lslidar_ls_driver_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp std_msgs sensor_msgs pcl_conversions)
ament_export_include_directories(include ${PCL_COMMON_INCLUDE_DIRS})

ament_package()


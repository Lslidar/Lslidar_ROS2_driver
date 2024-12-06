cmake_minimum_required(VERSION 3.5)
project(lslidar_ch_driver)

set(CMAKE_BUILD_TYPE Release)    #RelWithDebInfo
add_definitions(-O3)
#add_compile_options(-std=c++11)
# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(libpcap_LIBRARIES -lpcap)

find_package(angles REQUIRED)
find_package(PCL REQUIRED)
#find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
   "msg/LslidarPacket.msg"
)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
)

rosidl_generate_interfaces(lslidar_soild
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)

add_executable(lslidar_ch_driver_node src/lslidar_driver_node.cpp src/lslidar_driver.cpp src/input.cpp)
target_link_libraries(lslidar_ch_driver_node ${rclcpp_LIBRARIES}  ${libpcap_LIBRARIES})
ament_target_dependencies(lslidar_ch_driver_node rclcpp std_msgs sensor_msgs pcl_conversions)#ament_index_cpp

install(DIRECTORY launch params rviz_cfg
        DESTINATION share/${PROJECT_NAME})

install(TARGETS
        lslidar_ch_driver_node
        DESTINATION lib/${PROJECT_NAME}
        )

ament_export_dependencies(rclcpp std_msgs sensor_msgs pcl_conversions)
ament_export_include_directories(include ${PCL_COMMON_INCLUDE_DIRS})

rosidl_target_interfaces(lslidar_ch_driver_node lslidar_soild "rosidl_typesupport_cpp")

ament_package()

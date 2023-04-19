#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os
import subprocess
import sys


def generate_launch_description():
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lslidar_cx126s3.yaml')
   # rviz_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'rviz_cfg', 'lslidar_cx128s2.rviz')

    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    driver_node = ""
   # rviz_node = ""
    ros_version = p.communicate()[0]
    print(ros_version)
    if ros_version == b'dashing\n' or ros_version == b'eloquent\n':
        print("ROS VERSION: dashing/eloquent")
        driver_node = LifecycleNode(package='lslidar_driver',
                                    node_namespace='cx126s3',
                                    node_executable='lslidar_driver_node',
                                    node_name='lslidar_driver_node',
                                    output='screen',
                                    parameters=[driver_dir],
                                    )
    elif ros_version == b'foxy\n' or ros_version == b'galactic\n' or ros_version == b'humble\n':
        print("ROS VERSION: foxy/galactic/humble")
        driver_node = LifecycleNode(package='lslidar_driver',
                                    namespace='cx126s3',
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    output='screen',
                                    emulate_tty=True,
                                    parameters=[driver_dir],
                                    )

    else:
        print("Please configure the ros environment")
        sys.exit()

    return LaunchDescription([
        driver_node
    ])

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


def generate_launch_description():
    driver_dir = os.path.join(get_package_share_directory('lslidar_ls_driver'), 'params', 'lslidar_ls1550.yaml')
    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    driver_node = ""
    ros_version = p.communicate()[0]
    print(ros_version)
    driver_node = LifecycleNode(package='lslidar_ls_driver',
                                namespace='ls1550',
                                executable='lslidar_ls_driver_node',
                                name='lslidar_ls_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[driver_dir],
                                )


    return LaunchDescription([
        driver_node
    ])
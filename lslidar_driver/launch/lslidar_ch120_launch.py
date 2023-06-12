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
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lslidar_ch120.yaml')
    rviz_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'rviz_cfg', 'lslidar.rviz')
    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE,shell=True)
    driver_node = ""
    rviz_node = ""
    result = p.communicate()[0]
    print(result)
    if result == b'dashing\n' or result == b'eloquent\n':
        print("ROS VERSION: dashing/eloquent")
        driver_node = LifecycleNode(package='lslidar_driver',
                                node_namespace='ch120',
                                node_executable='lslidar_driver_node',
                                node_name='lslidar_driver_node',
                                output='screen',
                                parameters=[driver_dir],
                                )
        rviz_node = Node(package='rviz2',
            node_namespace='ch120',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_dir],
            output='screen')
    elif result == b'foxy\n' or result == b'galactic\n' or result == b'humble\n':
        print("ROS VERSION: foxy/galactic/humble")
        driver_node = LifecycleNode(package='lslidar_driver',
                                namespace='ch120',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[driver_dir],
                                )

        rviz_node = Node(
            package='rviz2',
            namespace='ch120',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_dir],
            output='screen')
    else:
        print("Please configure the ros environment")
    return LaunchDescription([
        driver_node , rviz_node
    ])

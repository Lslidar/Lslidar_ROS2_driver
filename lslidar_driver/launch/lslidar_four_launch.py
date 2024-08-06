#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import os
import subprocess

def generate_launch_description():
    driver_dir_front = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lslidar_cx_front.yaml')
    driver_dir_rear = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lslidar_cx_rear.yaml')
    driver_dir_left = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lslidar_cx_left.yaml')
    driver_dir_right = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lslidar_cx_right.yaml')

    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    driver_node_front = ""
    driver_node_rear = ""
    driver_node_left = ""
    driver_node_right = ""

    ros_version = p.communicate()[0]
    print(ros_version)
    if ros_version == b'dashing\n' or ros_version == b'eloquent\n':
        print("ROS VERSION: dashing/eloquent")
        driver_node_front = LifecycleNode(package='lslidar_driver',
                                         node_namespace='cx_front',
                                         node_executable='lslidar_driver_node',
                                         node_name='lslidar_driver_node',
                                         output='screen',
                                         parameters=[driver_dir_front],
                                         )

        driver_node_rear = LifecycleNode(package='lslidar_driver',
                                          node_namespace='cx_rear',
                                          node_executable='lslidar_driver_node',
                                          node_name='lslidar_driver_node',
                                          output='screen',
                                          parameters=[driver_dir_rear],
                                          )
        
        driver_node_left = LifecycleNode(package='lslidar_driver',
                                         node_namespace='cx_left',
                                         node_executable='lslidar_driver_node',
                                         node_name='lslidar_driver_node',
                                         output='screen',
                                         parameters=[driver_dir_left],
                                         )

        driver_node_right = LifecycleNode(package='lslidar_driver',
                                          node_namespace='cx_right',
                                          node_executable='lslidar_driver_node',
                                          node_name='lslidar_driver_node',
                                          output='screen',
                                          parameters=[driver_dir_right],
                                          )

    elif ros_version == b'foxy\n' or ros_version == b'galactic\n' or ros_version == b'humble\n':
        print("ROS VERSION: foxy/galactic/humble")
        driver_node_front = LifecycleNode(package='lslidar_driver',
                                         namespace='cx_front',
                                         executable='lslidar_driver_node',
                                         name='lslidar_driver_node',
                                         output='screen',
                                         emulate_tty=True,
                                         parameters=[driver_dir_front],
                                         )

        driver_node_rear = LifecycleNode(package='lslidar_driver',
                                          namespace='cx_rear',
                                          executable='lslidar_driver_node',
                                          name='lslidar_driver_node',
                                          output='screen',
                                          emulate_tty=True,
                                          parameters=[driver_dir_rear],
                                          )
        
        driver_node_left = LifecycleNode(package='lslidar_driver',
                                         namespace='cx_left',
                                         executable='lslidar_driver_node',
                                         name='lslidar_driver_node',
                                         output='screen',
                                         emulate_tty=True,
                                         parameters=[driver_dir_left],
                                         )

        driver_node_right = LifecycleNode(package='lslidar_driver',
                                          namespace='cx_right',
                                          executable='lslidar_driver_node',
                                          name='lslidar_driver_node',
                                          output='screen',
                                          emulate_tty=True,
                                          parameters=[driver_dir_right],
                                          )

    else:
        print("Please configure the ros environment")
        exit()

    return LaunchDescription([
        driver_node_front,
        driver_node_rear,
        driver_node_left,
        driver_node_right
    ])
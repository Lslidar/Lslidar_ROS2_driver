import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    left_driver_config = os.path.join(get_package_share_directory('lslidar_driver'),'config','lslidar_cx_left.yaml')
    right_driver_config = os.path.join(get_package_share_directory('lslidar_driver'),'config','lslidar_cx_right.yaml')

    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    left_driver_node = ""
    right_driver_node = ""
    ros_version = p.communicate()[0]

    if ros_version == b'dashing\n' or ros_version == b'eloquent\n':
        left_driver_node = LifecycleNode(package='lslidar_driver',
                                    node_executable='lslidar_driver_node',
                                    node_name='lslidar_driver_node',
                                    node_namespace='cx_left', # 与对应yaml文件中命名空间一致
                                    output='screen',
                                    parameters=[left_driver_config],
                                    )
        
        right_driver_node = LifecycleNode(package='lslidar_driver',
                                    node_executable='lslidar_driver_node',
                                    node_name='lslidar_driver_node',
                                    node_namespace='cx_right', # 与对应yaml文件中命名空间一致
                                    output='screen',
                                    parameters=[right_driver_config],
                                    )

    else:
        left_driver_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',
                                namespace='cx_left', # 与对应yaml文件中命名空间一致
                                parameters=[left_driver_config],
                                output='screen'
                                )
    
        right_driver_node = LifecycleNode(package='lslidar_driver',
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    namespace='cx_right', # 与对应yaml文件中命名空间一致
                                    parameters=[right_driver_config],
                                    output='screen'
                                    )


    return LaunchDescription([
        left_driver_node,
        right_driver_node
    ])

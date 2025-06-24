import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    driver_config = os.path.join(get_package_share_directory('lslidar_driver'),'config','lslidar_ch.yaml')
    rviz_config = os.path.join(get_package_share_directory('lslidar_driver'),'rviz','lslidar_ch.rviz')

    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    driver_node = ""
    rviz_node = ""
    ros_version = p.communicate()[0]

    if ros_version == b'dashing\n' or ros_version == b'eloquent\n':
        driver_node = LifecycleNode(package='lslidar_driver',
                                    node_executable='lslidar_driver_node',
                                    node_name='lslidar_driver_node',
                                    node_namespace='ch', # 与对应yaml文件中命名空间一致
                                    output='screen',
                                    parameters=[driver_config],
                                    )
        rviz_node = Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            node_namespace='cx',
            arguments=['-d', rviz_config],
            output='screen')
    else:
        driver_node = LifecycleNode(package='lslidar_driver',
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    namespace='ch', # 与对应yaml文件中命名空间一致
                                    parameters=[driver_config],
                                    output='screen'
                                    )
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )

    return LaunchDescription([
        driver_node,
        rviz_node
    ])

### LSLIDAR_CH1W_V1.0.0_240806_ROS2驱动说明

## 1.工程介绍

​		LSLIDAR_CH1W_V1.0.0_240806_ROS2为linux环境下雷达ros2驱动，适用于镭神ch1w雷达。

## 2.依赖

1.ubuntu18.04 ros dashing/ubuntu18.04 ros eloquent/ubuntu 20.04 ros foxy/ubuntu 20.04 ros galactic/ubuntu22.04 ros humble

2.ros依赖

```bash
# 安装
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions 
```

3.其他依赖

pcap

~~~bash
sudo apt-get install libpcap-dev
~~~



## 3.编译与运行：

~~~shell
mkdir -p ~/lslidar_ws/src
cd ~/lslidar_ws/src
把驱动压缩包拷贝到src目录下，并解压
cd ~/lslidar_ws
colcon build
source install/setup.bash

#启动单个ch1w雷达
ros2 launch lslidar_driver lslidar_ch1w_launch.py
#启动两个ch1w雷达
ros2 launch lslidar_driver lslidar_ch1w_double_launch.py
#启动四个ch1w雷达
ros2 launch lslidar_driver lslidar_ch1w_four_launch.py
~~~



## 4.launch 文件参数说明：

~~~shell
/ch1w/lslidar_ch_driver_node:
  ros__parameters:
    lidar_type: CH1W                    # 雷达类型
    device_ip: 192.168.1.200            # 雷达IP
    msop_port: 2368                     # 雷达目的数据端口
    difop_port: 2369                    # 雷达目的设备端口
    pcl_type: false                     # 发布点云类型 true: xyzi
    add_multicast: false                # 雷达是否为组播模式
    group_ip: 224.1.1.2                 # 组播IP
    use_time_service: false             # 雷达是否使用GPS/PTP/NTP授时
    min_range: 0.20                     # 雷达扫描最小距离,小于此值的点将被驱动过滤
    max_range: 500.0                    # 雷达扫描最大距离,大于此值的点将被驱动过滤
    scan_start_angle: 0                 # 雷达扫描最小角度,小于此值的点将被驱动过滤 单位: 0.01°
    scan_end_angle: 18000               # 雷达扫描最大角度,大于此值的点将被驱动过滤 单位: 0.01°
    frame_id: laser_link                # 雷达点云帧ID
    topic_name: lslidar_point_cloud     # 雷达点云话题名称
    echo_num: 0                         # 仅双回波模式下有效，0表示发布所有点云，1表示发布第一次回波点云，2表示发布第二次回波点云
    publish_scan: false                 # 驱动是否发布laserscan
    horizontal_angle_resolution: 0.024  # laserscan 角度分辨率 20hz: 0.012  80hz: 0.048
    packet_rate: 3510.0                 # 离线加载PCAP包,每秒PCAP包数
    #pcap: /home/ls/data/xxx.pcap       # PCAP包路径，加载PCAP包时打开此注释

~~~

### 组播模式：

- 上位机设置雷达开启组播模式

- 修改launch文件以下参数

  ~~~shell
  add_multicast: true                    #是否开启组播模式。
  group_ip: 224.1.1.2                     #组播ip地址
  ~~~

- 运行以下指令将电脑加入组内（将指令中的enp2s0替换为用户电脑的网卡名,可用ifconfig查看网卡名)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### 离线pcap模式：

- 修改launch文件以下参数

  ~~~shell
  #取消注释
  pcap: /home/chris/Documents/leishen/lidar.pcap                        #pcap包路径，加载pcap包时打开此注释
  ~~~



###  pcl点云类型：

- 修改launch文件以下参数

  ~~~shell
  pcl_type: false                         #点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
  ~~~

  

- 默认false为自定义点云类型，定义参考lslidar_driver/include/lslidar_driver.h头文件

- 改为true,为pcl自带类型 :

  ~~~shell
  pcl::PointCloud<pcl::PointXYZI>
  ~~~

  

## FAQ

Bug Report

Original version : LSLIDAR_CH1W_V1.0.0_240806_ROS2

Modify:  original version

Date    : 2024-08-06

---------------

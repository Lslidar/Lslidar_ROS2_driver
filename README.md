# LSLIDAR_CHX1_V1.0.3_240621_ROS2驱动说明

## 1.工程介绍

​		LSLIDAR_CHX1_V1.0.3_240621_ROS2为linux环境下雷达ros2驱动，适用于镭神CX1S3,CX6S3,CH16X1,CH64W,CB64S1-A,CH128X1,CH128S1,CX126S3,CX128S2,CH256的雷达 ，程序在ubuntu18.04 ros dashing , ubuntu18.04 ros eloquent ,ubuntu 20.04 ros foxy,ubuntu 20.04 ros galactic以及ubuntu22.04 ros humble下测试通过。

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
#创建工作空间
mkdir -p ~/lslidar_ws/src
#把驱动压缩包拷贝到src目录下，并解压
cd ~/lslidar_ws/src
unzip LSLIDAR_CHX1_V1.0.2_231101_ROS2
#返回工作空间
cd ~/lslidar_ws
#编译	           / 指定编译驱动功能包   选其一
colcon build	/ colcon build --packages-select lslidar_ch_driver

#刷新当前终端环境
source install/setup.bash
#启动单个激光雷达雷达
ros2 launch lslidar_ch_driver lslidar_ch_launch.py

#启动单个激光雷达雷达并加载RVIZ2
ros2 launch lslidar_ch_driver lslidar_ch_rviz_launch.py

#启动两个激光雷达雷达
ros2 launch lslidar_ch_driver lslidar_double_launch.py 
~~~



## 4.lslidar_ch.yaml文件参数说明：

~~~shell
/CH/lslidar_ch_driver_node:
  ros__parameters:
    lidar_type: CX128S2               # 雷达类型  CX1S3/CX6S3/CH16X1/CH64W/CB64S1-A/CH128X1/CH128S1/CX126S3/CX128S2/CH256	雷达类型不能错
    device_ip: 192.168.1.200          # 雷达IP
    msop_port: 2368                   # 雷达目的数据端口
    difop_port: 2369                  # 雷达目的设备端口
    add_multicast: false              # 雷达是否为组播模式
    group_ip: 224.1.1.2               # 组播IP
    use_time_service: true            # 雷达是否使用GPS/PTP/NTP授时
    min_range: 0.20                   # 雷达扫描最小距离,小于此值的点将被驱动过滤 可根据实际需求更改
    max_range: 500.0                  # 雷达扫描最大距离,大于此值的点将被驱动过滤 可根据实际需求更改
    scan_start_angle: 3000            # 雷达扫描最小角度,小于此值的点将被驱动过滤 单位: 0.01°    		 									64线雷达改为0 	 可根据实际需求更改 
    scan_end_angle: 15000             # 雷达扫描最大角度,大于此值的点将被驱动过滤 单位: 0.01°  											64线雷达改为18000 可根据实际需求更改 
    frame_id: laser_link              # 雷达点云帧ID		
    topic_name: lslidar_point_cloud   # 雷达点云话题名称
    echo_num: 0                       # 仅双回波模式下有效，0表示发布所有点云，1表示发布第一次回波点云，2表示发布第二次回波点云
    publish_scan: false               # 驱动是否发布laserscan
    channel_num: 8                    # 发布laserscan的线号
    horizontal_angle_resolution: 0.2  # 10Hz:0.2  20Hz:0.4  5Hz: 0.1
    packet_rate: 9826.0               # 离线加载PCAP包,每秒PCAP包数，每款雷达每秒包数不一样
    #pcap: /home/ls/data/xxx.pcap     # PCAP包路径，离线解析PCAP包时打开此注释

~~~



### 组播模式：

- 上位机设置雷达开启组播模式

- 修改yaml文件以下参数

  ~~~shell
  add_multicast: true                    # 是否开启组播模式。
  group_ip: 224.1.1.2                    # 组播ip地址
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
  pcap: /home/ls/data/xxx.pcap        # pcap包路径，加载pcap包时打开此注释
  ~~~



###  pcl点云类型：

- 自定义点云类型，定义参考/include/lslidar_ch_driver/lslidar_driver.h头文件

  ~~~c++
  lslidar_ch_driver::PointXYZIRT,
                      (float, x, x)					// x 坐标
                      (float, y, y)					// y 坐标
                      (float, z, z)					// z 坐标
                      (float, intensity, intensity)	// 强度值
                      (std::uint8_t, ring, ring)		// 线号
                      (float, time, time))			// 时间(相对)
  ~~~

  

## FAQ

Bug Report

Original version : LSLIDAR_CHX1_V1.0.0_221116_ROS2

Modify:  original version

Date    : 2022-11-16

---------------



Update version : LSLIDAR_CHX1_V1.0.1_230316_ROS2

Modify:  

降低cpu占用；点的时间改为相对时间；新增兼容cx128s2 雷达。

Date    : 2023-03-16

---------------



Update version : LSLIDAR_CHX1_V1.0.2_230419_ROS2

Modify:  

新增兼容cx128s2 雷达;解析package.xml打印驱动版本。

Date    : 2023-04-19

----



Update version : LSLIDAR_CHX1_V1.0.3_240621_ROS2

Modify:  

​			1.合并功能包

​			2.优化代码，降低CPU占用。

​			3.新CX128S2雷达水平角度计算。

​			4.新增兼容CX1S3,CX6S3,CB64S1-A雷达。

Date    : 2024-06-21

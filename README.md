# LSLIDAR_LS1550_V1.0.6_240910_ROS2使用说明

## 1.工程介绍
​		LSLIDAR_LS1550_V1.0.6_240910_ROS2为linux环境下雷达ros驱动，适用于LS25D、LS128、LS144、LS160、LS180、LS320、LS400、MS06系列雷达，程序在ubuntu 20.04 ros foxy 和 ubuntu 20.04 ros galactic 和 ubuntu 22.04 ros humble下测试通过。



## 2.依赖

1.ubuntu 20.04 ros foxy/ubuntu 20.04 ros galactic/ubuntu 22.04 ros humble

2.ros依赖

```bash
# 安装
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions  
```

3.其他依赖

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #选择适合的版本
~~~



## 3.编译运行

### 3.1 编译

~~~bash
mkdir -p ~/lidar_ws/src
#ROS2压缩包解压缩放到~/lidar_ws/src 目录下
cd ~/lidar_ws
colcon build
source install/setup.bash
~~~

### 3.2 运行

运行单个雷达:

~~~bash
ros2 launch lslidar_ls_driver lslidar_ls1550_launch.py
ros2 launch lslidar_ls_driver lslidar_ls1550_rviz_launch.py		# 同时加载 RVIZ
~~~

运行多个雷达：

~~~bash
ros2 launch lslidar_ls_driver lslidar_ls1550_double_launch.py
~~~

### 4.参数介绍

lslidar_ls1550.yaml文件内容如下，每个参数含义见注释说明。

~~~bash
/ls1550/lslidar_ls_driver_node:
  ros__parameters:
    device_ip: 192.168.1.200    #雷达ip
    msop_port: 2368             #雷达目的数据端口
    difop_port: 2369            #雷达目的设备端口
    frame_id: laser_link        #雷达点云帧id
    add_multicast: false        #是否开启组播
    group_ip: 224.1.1.2         #组播ip
    time_synchronization: false #是否使用授时
    min_range: 0.3              #点云最小距离裁剪
    max_range: 500.0            #点云最大距离裁剪
    scan_start_angle: -60       # 起始角度，范围[-60,60]
    scan_end_angle: 60          # 结束角度，范围[-60,60]
    topic_name: lslidar_point_cloud #点云话题
    packet_loss: false          #是否开启丢包检测
    packet_rate: 10000.0        #pcap包每秒包数
    #pcap: /home/ls/xxx.pcap    #pcap包路径，加载pcap包时打开此注释
~~~



### 组播模式：

- 上位机设置雷达开启组播模式

- 修改launch文件以下参数

  ~~~shell
  add_multicast: true                    #是否开启组播模式。
  group_ip: 224.1.1.2                    #组播ip地址
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
  pcap: /home/ls/work/fuhong/data/ls128/20221012-ntp.pcap                         #pcap包路径，加载pcap包时打开此注释
  ~~~



## 5.丢包检测

驱动将雷达丢包总数以topic的形式发布出来，topic名字为 ***packet_loss***，消息类型为std_msgs::msg::Int64



## 6.雷达故障码

驱动将雷达故障码以topic的形式发布出来，topic名字为***lslidar_fault_code***，消息类型为std_msgs::msg::String

*每位故障码含义请联系技术支持*



## 7.设置雷达

### 设置雷达ip

source install/setup.bash

~~~bash
ros2 service call /xx/set_data_ip lslidar_ls_driver/srv/DataIp "{data_ip: '192.168.1.200'}" #说明：xx为命名空间，例如/ls1550
~~~

**备注：设置完以后，需要修改yaml文件参数，然后重启驱动程序。**



### 设置雷达目的ip

source install/setup.bash

~~~bash
ros2 service call /xx/set_destination_ip lslidar_ls_driver/srv/DestinationIp "{destination_ip: '192.168.1.102'}"   #说明：xx为命名空间，例如/ls1550
~~~

**备注：设置完以后，需要修改本地ip，然后重启驱动程序。**



### 设置雷达数据包端口

source install/setup.bash

~~~bash
ros2 service call /xx/set_data_port lslidar_ls_driver/srv/DataPort "{data_port: 2368}"  #范围[1025,65535]   #说明：xx为命名空间，例如/ls1550
~~~

**备注：设置完以后，需要修改yaml文件参数，然后重启驱动程序。**



### 设置雷达设备包端口

source install/setup.bash

~~~bash
ros2 service call /xx/set_dev_port lslidar_ls_driver/srv/DevPort "{dev_port: 2369}"   #范围[1025,65535]   #说明：xx为命名空间，例如/ls1550
~~~

**备注：设置完以后，需要修改yaml文件参数，然后重启驱动程序**



### 设置雷达帧率(支持修改帧率的雷达)

source install/setup.bash

~~~bash
ros2 service call /xx/set_frame_rate lslidar_ls_driver/srv/FrameRate "{frame_rate: 0}"   #范围[0，25，50]   #说明：xx为命名空间，例如/ls1550  0为标准帧率  25为标准帧率的25%  50为标准帧率的50%
~~~



### 设置雷达授时方式

source install/setup.bash

**GPS授时**

~~~bash
ros2 service call /xx/set_time_service lslidar_ls_driver/srv/TimeService "{time_service_mode: 'gps',ntp_ip: ''}"	 	#说明：xx为命名空间，例如/ls1550
~~~

**PTP授时**

~~~bash
# PTP L2
ros2 service call /xx/set_time_service lslidar_ls_driver/srv/TimeService "{time_service_mode: 'ptp_l2',ntp_ip: ''}"  	#说明：xx为命名空间，例如/ls1550

# PTP UDPV4
ros2 service call /xx/set_time_service lslidar_ls_driver/srv/TimeService "{time_service_mode: 'ptp_udpv4',ntp_ip: ''}"  #说明：xx为命名空间，例如/ls1550
~~~

**NTP授时**

~~~sh
ros2 service call /xx/set_time_service lslidar_ls_driver/srv/TimeService "{time_service_mode: 'ntp',ntp_ip: '192.168.1.102'}"   #说明：192.168.1.102为NTP ip
~~~

**备注：修改后请修改yaml文件中 *time_synchronization* 参数改为 *true***



### 设置雷达待机模式

source install/setup.bash

~~~bash
ros2 service call /xx/set_standby_mode lslidar_ls_driver/srv/StandbyMode "{standby_mode: 0}"   #说明：xx为命名空间，例如/ls1550	0: 正常模式		1: 待机模式
~~~



### 设置雷达角度畸变校正

source install/setup.bash

~~~bash
ros2 service call /ls1550/set_angle_distortion_correction lslidar_ls_driver/srv/AngleDistortionCorrection "{angle_distortion_correction: 1}"
#说明：xx为命名空间，例如/ls1550	0: 关闭		1: 开启
~~~

**备注：修改后请重启驱动程序**



### 设置雷达是否发布无效数据

source install/setup.bash

~~~bash
ros2 service call /ls1550/set_invalid_data lslidar_ls_driver/srv/InvalidData "{invalid_data: 1}"
#说明：xx为命名空间，例如/ls1550	0: 发送		1: 不发送
~~~





## FAQ

Bug Report

Original version : LSLIDAR_LS128_ROS_V1.0.1_221012_ROS2

Modify:  original version

Date    : 2022-10-12

----------------



Update version : LSLIDAR_LS128_ROS_V1.0.1_221128_ROS2

Modify:  1.fpga版本升级，修改点云计算公式

​                2.新增对ros2 humble, ros2 dashing, ros2 eloquent的支持

Date    : 2022-11-28

---------------



Update version : LSLIDAR_LS128_ROS_V1.0.2_230301_ROS2

Modify:  1.新增对LS128S2雷达的支持

​                2.fpga协议变更，修改叠帧判断条件

Date    : 2023-03-01

-----------------



Update version : LSLIDAR_LS128_ROS_V1.0.3_230313_ROS2

Modify:  1.预计算正弦余弦值，减少CPU占用

​			   2.新增丢包检测的功能

Date    : 2023-03-13

-----------------



Update version : LSLIDAR_LS1550_ROS_V1.0.4_240308_ROS2

Modify:  1.兼容MS06雷达

​			   2.增加设置雷达服务

Date    : 2024-03-08

----



Update version : LSLIDAR_LS1550_V1.0.5_240808_ROS2

Modify:  1.优化代码结构

​				2.新增故障码发布

​			    2.合并消息与服务功能包

​				3.兼容雷达角度畸变校正解析

​				4.新增设置修改雷达时间源，待机模式，角度畸变矫正，发送无效数据服务

Date    : 2024-08-08

----



Update version : LSLIDAR_LS1550_V1.0.6_240910_ROS2

Modify:  1.更新点云数据的内存分配方式，避免 boost 的兼容性问题

Date    : 2024-09-10

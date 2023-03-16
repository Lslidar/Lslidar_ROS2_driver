### LSLIDAR_CHX1_V1.0.0_221116_ROS2驱动说明

## 1.工程介绍

​		LSLIDAR_CHX1_V1.0.0_221116_ROS2为linux环境下雷达ros2驱动，适用于镭神ch16x1/ch64w/ch128x1/ch128s1/cx128s2的雷达 ，程序在ubuntu18.04 ros dashing , ubuntu18.04 ros eloquent ,ubuntu 20.04 ros foxy,ubuntu 20.04 ros galactic以及ubuntu22.04 ros humble下测试通过。

## 2.依赖

1.ubuntu18.04 ros dashing/ubuntu18.04 ros eloquent/ubuntu 20.04 ros foxy/ubuntu 20.04 ros galactic/ubuntu22.04 ros humble

2.ros依赖

```bash
# 安装
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions 
```

3.其他依赖

pcap,boost

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #选择适合的版本
~~~



## 3.编译与运行：

~~~shell
mkdir -p ~/lslidar_ws/src
cd ~/lslidar_ws/src
把驱动压缩包拷贝到src目录下，并解压
cd ~/lslidar_ws
colcon build
source install/setup.bash
#启动单个ch16x1雷达
ros2 launch lslidar_driver lslidar_ch16x1_launch.py
#启动两个ch16x1雷达
ros2 launch lslidar_driver lslidar_ch16x1_double_launch.py

#启动单个ch64w雷达
ros2 launch lslidar_driver lslidar_ch64w_launch.py
#启动两个ch64w雷达
ros2 launch lslidar_driver lslidar_ch64w_double_launch.py

#启动单个ch128x1雷达
ros2 launch lslidar_driver lslidar_ch128x1_launch.py
#启动两个ch128x1雷达
ros2 launch lslidar_driver lslidar_ch128x1_double_launch.py

#启动单个ch128s1雷达
ros2 launch lslidar_driver lslidar_ch128s1_launch.py
#启动两个ch128s1雷达
ros2 launch lslidar_driver lslidar_ch128s1_double_launch.py
~~~



## 4.launch 文件参数说明：

~~~shell
/ch16x1/lslidar_driver_node:
  ros__parameters:
    device_ip: 192.168.1.200        #雷达ip
    lidar_type: ch16x1              #雷达类型  ch16x1/ch64w/ch128x1/ch128s1/cx128s2
    msop_port: 2368                 #数据包目的端口
    difop_port: 2369                #设备包目的端口
    pcl_type: false                 #点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
    add_multicast: false            #是否开启组播模式
    group_ip: 224.1.1.2             #组播ip地址
    frame_id: laser_link            #坐标系id
    min_range: 0.3                  #单位，米。雷达盲区最小值，小于此值的点被过滤
    max_range: 200.0                #单位，米。雷达盲区最大值 ，大于此值的点被过滤
    packet_rate: 1409.0             #播放pcap时，每秒钟播放的包数
    angle_disable_min: 0            #雷达裁剪角度开始值 ，范围[0,18000]
    angle_disable_max: 0            #雷达裁剪角度结束值，范围[0,18000]
    topic_name: lslidar_point_cloud         #点云话题名称，可修改
    horizontal_angle_resolution: 0.2     #10Hz:0.2  20Hz:0.4 5Hz: 0.1
    use_time_service: true                #是否使用GPS/ptp/ntp授时
    echo_num: 0                          #仅双回波模式下有效，0表示发布所有点云，1表示发布第一次回波点云，2表示发布第二次回波点云
    publish_scan: false                     #是否发布scan
    scan_num: 15                            #laserscan线号
    #pcap: /home/ls/work/xxx.pcap                        #pcap包路径，加载pcap包时打开此注释
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

Original version : LSLIDAR_CHX1_V1.0.0_221116_ROS2

Modify:  original version

Date    : 2022-11-16

---------------



Update version : LSLIDAR_CHX1_V1.0.1_230316_ROS2

Modify:  

降低cpu占用；点的时间改为相对时间；新增兼容cx128s2 雷达。

Date    : 2023-03-16

---------------




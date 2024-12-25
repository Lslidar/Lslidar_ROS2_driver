### LSLIDAR_CH1W_V1.0.0_240806_ROS2 Driver Descriptions

## 1.Introduction

​		LSLIDAR_CH1W_V1.0.0_240806_ROS2 is the lidar ros driver under Linux system, applicable to CH1W lidar. 

## 2.Dependancies

1.Ubuntu18.04 ros dashing/ubuntu18.04 ros eloquent/ubuntu 20.04 ros foxy/ubuntu 20.04 ros galactic/ubuntu22.04 ros humble

2.ROS dependancy

```bash
# installation
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions 
```

3.Other dependancy

pcap

~~~bash
sudo apt-get install libpcap-dev
~~~



## 3.Compile&Run:

~~~shell
mkdir -p ~/lslidar_ws/src
cd ~/lslidar_ws/src
Copy the whole the driver zip file to src directory and then unzip
cd ~/lslidar_ws
colcon build
source install/setup.bash

#start single ch1w lidar
ros2 launch lslidar_driver lslidar_ch1w_launch.py
#start 2 ch1w lidars
ros2 launch lslidar_driver lslidar_ch1w_double_launch.py
#start 4 ch1w lidars
ros2 launch lslidar_driver lslidar_ch1w_four_launch.py
~~~



## 4. Launch file parameters decriptions 

~~~shell
/ch1w/lslidar_ch_driver_node:
  ros__parameters:
    lidar_type: CH1W                    # lidar type
    device_ip: 192.168.1.200            # lidar IP
    msop_port: 2368                     # MSOP (Main data Stream Output Protocol) packet port 
    difop_port: 2369                    # DIFOP (Device Information Output Protocol) packet port 
    pcl_type: false                     # publish point clout type:true: xyzi
    add_multicast: false                # Whether lidar in multicast mode
    group_ip: 224.1.1.2                 # multicast IP
    use_time_service: false             # Whether lidar adopts GPS/PTP/NTP time synchronization
    min_range: 0.20                     # The minimum value of the lidar blind area, points less than this value are filtered
    max_range: 500.0                    # The maximum value of the lidar blind area, points greater than this value are filtered
    scan_start_angle: 0                 # The minimum angle for of lidar scanning, points less than this value will be filtered; unit: 0.01°
    scan_end_angle: 18000               # The maximum angle for of lidar scanning, points greater than this value will be filtered; unit: 0.01°
    frame_id: laser_link                # Point cloud frame ID
    topic_name: lslidar_point_cloud     # Point clou topic name
    echo_num: 0                         # Only valid in dual echo mode, 0 means publishing all point cloud, 1 means publishing the 1st echo, 2 means publishing the 2nd echo.  
    publish_scan: false                 # whether driver publishlaserscan
    horizontal_angle_resolution: 0.024  # laserscan angular resolution 20hz: 0.012  80hz: 0.048
    packet_rate: 3510.0                 # loading of PCAP packets, number of PCAP packets per second
    #pcap: /home/ls/data/xxx.pcap       # PCAP file path, open this comment when load pcap packet 

~~~

### Multicast mode:

- Set the multicast mode for the lidar on the windows client software 

- Modify the parameters in launch file  

  ~~~shell
  add_multicast: true                    #whether add multicast mode
  group_ip: 224.1.1.2                     #multicast IP addresss
  ~~~

- runt the command below to add the computer to the group (replace the  enp2s0 with the user's computer network adapter name,  you can use ifconfig to check the network adapter name) 

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### Offline PCAP mode: 

- modify  the parametrs in the launch file  

  ~~~shell
  #uncomment
  pcap: /home/chris/Documents/leishen/lidar.pcap                        #pcap packet file path, open this comment when load pcap packet   
  ~~~



###  pcl point cloud type:

- Modify the parameters in the launch file  

  ~~~shell
  pcl_type: false                         #point cloud type, by default, false means the point is xyzirt field, true means that the point is xyzi field. 
  ~~~

  

- By default: false means custom point cloud type, for the definition, refer to lslidar_driver/include/lslidar_driver.h head file

- change it to true, it is pcl  built-in type:

  ~~~shell
  pcl::PointCloud<pcl::PointXYZI>
  ~~~

  

## FAQ

Bug Report

Original version : LSLIDAR_CH1W_V1.0.0_240806_ROS2

Modify:  original version

Date    : 2024-08-06

---------------

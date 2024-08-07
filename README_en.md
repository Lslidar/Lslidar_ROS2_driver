# LSLIDAR_CX_V4.2.4_240410_ROS2

## 1.Introduction
​		LSLIDAR_CX_V4.2.4_240410_ROS2 is the lidar ros driver in linux environment,.The program has been tested successfully under ubuntu 22.04 ROS humble,ubuntu20.04 ROS foxy, and ubuntu20.04 ROS galactic, ubuntu18.04 ROS dashing, and ubuntu18.04 ROS eloquent.

​		Applicable to C16,C32 3.0 version and C1, C1Plus, C4, C8, C8f, CKM8, C16, MSC16, C16 domestic, C32, C32W, C32WN,C32Wb,C32WP, CH32R 4.0 version and n301 5.5 version lidars.

## 2.Dependencies

1.ros

To run lidar driver in ROS environment, ROS related libraries need to be installed.

**Ubuntu 18.04**: ros-dashing-desktop-full

**Ubuntu 18.04**: ros-eloquent-desktop-full

**Ubuntu 20.04**: ros-foxy-desktop-full

**Ubuntu 20.04**: ros-galactic-desktop-full

**Ubuntu 22.04**: ros-humble-desktop-full

**Installation**: please refer to [http://wiki.ros.org](http://wiki.ros.org/)

2.ros dependencies

```bash
# install
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions 
```

3.other dependencies

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #Select the appropriate version
~~~

## 3.Compile & Run

### 3.1 Compile

~~~bash
mkdir -p ~/lidar_ws/src
~~~

Copy the whole lidar ROS driver directory into ROS workspace, i.e "~/lidar_ws/src".

~~~bash
cd ~/lidar_ws
colcon build
source install/setup.bash
~~~

### 3.2 Run

run with single lidar:

~~~bash
ros2 launch lslidar_driver lslidar_cx_launch.py
ros2 launch lslidar_driver lslidar_cx_rviz_launch.py		#Automatically load rviz
~~~

run with double lidar:

~~~bash
ros2 launch lslidar_driver lslidar_double_launch.py
ros2 launch lslidar_driver lslidar_double_rviz_launch.py	#Automatically load rviz
~~~

run with four lidar:

~~~bash
ros2 launch lslidar_driver lslidar_four_launch.py
~~~



## 4. Introduction to parameters

The content of the lslidar_cx.yaml file is as follows, and the meaning of each parameter is shown in the notes.

~~~bash
/cx/lslidar_driver_node:
  ros__parameters:
    packet_rate: 1695.0            #Number of packets played per second when playing pcap
    device_ip: 192.168.1.200       # lidar ip
    msop_port: 2368                # Main data Stream Output Protocol packet port
    difop_port: 2369               # Device Information Output Protocol packet port
    pcl_type: false                # pointcloud type，false: xyzirt,true:xyzi
    add_multicast: false           # Whether to add multicast
    group_ip: 224.1.1.2            # multicast ip
    use_time_service: true         # Whether gps time synchronization
    min_range: 0.15                # Unit: m. The minimum value of the lidar blind area, points smaller than this value are filtered
    max_range: 200.0               # Unit: m. The maximum value of the lidar blind area, points smaller than this value are filtered
    angle_disable_min: 0           # lidar clipping angle start value ，unit:0.01° Fill in integers
    angle_disable_max: 0           # lidar clipping angle end value ，unit:0.01° Fill in integers
    distance_unit: 0.4             # lidar range resolution
    horizontal_angle_resolution: 0.18  #10Hz:0.18  20Hz:0.36  5Hz: 0.09
    frame_id: laser_link           # lidar point cloud coordinate system name
    topic_name: lslidar_point_cloud     # point cloud topic name, can be modified
    publish_scan: false                 # Whether to publish the scan
    scan_num: 15                   # laserscan line number
    coordinate_opt: false               # Default false. The zero degree angle of the lidar corresponds to the direction of the point cloud
    #pcap: /home/ls/Documents/xxx.pcap  # Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar
~~~

### Multicast mode:

- The host computer sets the lidar to enable multicast mode

- Modify the following parameters of the launch file

  ~~~xml
  add_multicast: true
  group_ip: 224.1.1.2    //The multicast ip address set by the host computer
  ~~~

- Run the following command to add the computer to the group (replace enp2s0 in the command with the network card name of the user's computer, you can use ifconfig to view the network card name)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### Offline pcap mode:

- Copy the recorded pcap file to the lslidar_ros/lslidar_driver/pcap folder

- Modify the following parameters of the launch file

  ~~~xml
  // uncomment
  pcap: xxx.pcap  // xxx.pcap is changed to the copied pcap file name
  ~~~



###  pcl point cloud type:

- Modify the following parameters of the launch file

  ~~~xml
  pcl_type: false      # pointcloud type，false: xyzirt,true:xyzi
  ~~~

- The default false is the custom point cloud type, which references the definition in the file of

  lslidar_driver/include/lslidar_driver.h

  Change it to true, which is the own type of pcl:

  ~~~c++
  pcl::PointCloud<pcl::PointXYZI>
  ~~~



### Modify lidar time service mode:

source install/setup.bash

GPS：

~~~bash
ros2 service call /xx/time_service lslidar_msgs/srv/TimeService "{time_service_mode: 'gps',ntp_ip: ''}"   #Note: xx is the namespace, such as cx
~~~

PTP：(lidar that supports this function)

~~~bash
ros2 service call /xx/time_service lslidar_msgs/srv/TimeService "{time_service_mode: 'ptp',ntp_ip: ''}"  #Note: xx is the namespace, such as cx
~~~

NTP：(lidar that supports this function)

~~~bash
ros2 service call /xx/time_service lslidar_msgs/srv/TimeService "{time_service_mode: 'ntp',ntp_ip: '192.168.1.102'}"   #Note: xx is the namespace, such as cx
~~~



### lidar power on/off(lidar still rotates,  only send equipment packets):

source install/setup.bash

power on：

~~~bash
ros2 service call /xx/lslidar_control lslidar_msgs/srv/LslidarControl "{laser_control: 1}"   #Note: xx is the namespace, such as cx
~~~

power off：

~~~bash
ros2 service call /xx/lslidar_control lslidar_msgs/srv/LslidarControl "{laser_control: 0}"   #Note: xx is the namespace, such as cx
~~~



### lidar rotates/stops rotating (motor stops rotating)：

source install/setup.bash

turn：

~~~bash
ros2 service call /xx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: 1}"   #Note: xx is the namespace, such as cx
~~~

Stop rotating：

~~~bash
ros2 service call /xx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: 0}"   #Note: xx is the namespace, such as cx
~~~



### Set lidar speed:

source install/setup.bash

Optional frequency  5Hz/10Hz/20Hz

~~~bash
ros2 service call /xx/set_motor_speed lslidar_msgs/srv/MotorSpeed "{motor_speed: 20}"   #Note: xx is the namespace, such as cx
~~~



### Set lidar data packet port

source install/setup.bash

~~~bash
ros2 service call /xx/set_data_port lslidar_msgs/srv/DataPort "{data_port: 2368}"  #range[1025,65535]   #Note: xx is the namespace, such as cx
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



### Set lidar equipment packet port

source install/setup.bash

~~~bash
ros2 service call /xx/set_dev_port lslidar_msgs/srv/DevPort "{dev_port: 2369}"#range[1025,65535]   #Note: xx is the namespace, such as cx
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



### Set lidar ip

source install/setup.bash

~~~bash
ros2 service call /xx/set_data_ip lslidar_msgs/srv/DataIp "{data_ip: "192.168.1.200"}"   #Note: xx is the namespace, such as cx
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



### Set lidar destination ip

source install/setup.bash

~~~bash
ros2 service call /xx/set_destination_ip lslidar_msgs/srv/DestinationIp "{destination_ip: "192.168.1.102"}"   #Note: xx is the namespace, such as cx
~~~





## FAQ

Bug Report

Original version : LSLIDAR_CX_V4.0.0_221031_ROS2

Modify:  original version

Date    : 2022-10-31

-----------------

Update version : LSLIDAR_CX_V4.1.0_221227_ROS2

Modify:1. Scan topic adds strength information
  2. fpga upgrade, C32 90 degree modification of calculation formula
  3. ROS driver adds the function of modifying time service mode
  4. New function to modify lidar configuration
  5. Added support for ros2 dashing, ros2 eloquent, and ros2 humble
  6. Fixed the ntp timing resolution problem.

Date    : 2022-12-27

-----------------



Update version : LSLIDAR_CX_V4.2.2_230322_ROS2

Modify:1.Prompt for usage duration

​			   2.Driver version prompt

Date    : 2023-03-22

-------------------



Update version : LSLIDAR_CX_V4.2.3_230403_ROS2

Modify: 1.Change the fpga protocol and modify the calculation formula of C32W

Date    : 2023-04-03

-------------------------



Update version : LSLIDAR_CX_V4.2.4_240410_ROS2

Modify:  1.Optimize code to reduce CPU usage
        	   2.supports negative angle cropping
        	   3.Delete the idar model parameters and rewrite the automatic recognition lidar model
        	   4.Compatible with C16 C32 3.0 version, C1 C1Plus C4 C8F CKM8 C16 domestic version C32WN C32WB 4.0 version, CH32R 4.8 version, N301 5.5 version lidar
      	     5.Solve the problem of continuous zero crossing angle between two adjacent data packet points in 3.0 lidar, and fix the inability of 3.0 lidar to power on and off
       	    6.Restrict the range of lidar IP settings and prohibit setting 224 network segments as lidar IPs
Date    : 2024-04-10

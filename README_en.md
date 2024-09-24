# LSLIDAR_LS1550_V1.0.6_240910_ROS2

## 1.Introduction
​		LSLIDAR_LS1550_V1.0.6_240910_ROS2 is the lidar ros driver in linux environment, which is suitable for LS25D、LS128、LS144、LS160、LS180、LS320、LS400、MS06 lidar. The program has tested under ubuntu 20.04 ros foxy , ubuntu 20.04 ros galactic and ubuntu 22.04 ros humble .



## 2.Dependencies

1.ros

To run lidar driver in ROS environment, ROS related libraries need to be installed.

**Ubuntu 20.04**: ros-foxy-desktop-full

**Ubuntu 20.04**: ros-galactic-desktop-full

**Ubuntu 22.04**: ros-humble-desktop-full

**Installation**: please refer to [http://wiki.ros.org](http://wiki.ros.org/)

2.ros dependencies

```bash
# install
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions 
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

3.2 Run

run with single lidar:

~~~bash
ros2 launch lslidar_ls_driver lslidar_ls1550_launch.py
ros2 launch lslidar_ls_driver lslidar_ls1550_rviz_launch.py		# Simultaneously load RVIZ
~~~

run with double lidar:

~~~bash
ros2 launch lslidar_ls_driver lslidar_ls1550_double_launch.py
~~~



## 4. Introduction to parameters

The content of the lslidar_ls1550.yaml file is as follows, and the meaning of each parameter is shown in the notes.

~~~bash
/ls1550/lslidar_ls_driver_node:
  ros__parameters:
    device_ip: 192.168.1.200          # lidar ip
    msop_port: 2368                   # Main data Stream Output Protocol packet port
    difop_port: 2369                  # Device Information Output Protocol packet port
    frame_id: laser_link              # Coordinate system ID
    add_multicast: false              # Whether to add multicast
    group_ip: 224.1.1.2               # multicast ip
    time_synchronization: false       # Whether gps time synchronization
    min_range: 0.3                    # Minimum measuring distance of lidar
    max_range: 500.0                  # Maximum measuring distance of lidar
    scan_start_angle: -60             # Scan start angle, range - 60 ° to 60 °
    scan_end_angle: 60                # Scan end angle,   range - 60 ° to 60 °
    topic_name: lslidar_point_cloud   # name of point cloud topic
    packet_loss: false          	  # Enable packet loss detection
    packet_rate: 10000.0       		  # PCAP packets per second
    #pcap: /home/ls/xxx.pcap          # Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar
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

- Modify the following parameters of the launch file

  ~~~xml
  // uncomment
  pcap: xxx.pcap  // xxx.pcap is changed to the copied pcap file name
  ~~~



## 5. Packet loss detection

Drive to publish the total number of lidar packet loss in the form of topic with the name of "packet_loss" and the message type of std_ msgs::msg::Int64。



## 6. Lidar fault code

The driver will publish the lidar fault codes as a topic named lslidar_fault_code, with the message type std_msgs::msg::String.

*For the meaning of each fault code, please contact technical support.*



## 7.set lidar

### Set lidar ip

source install/setup.bash

~~~bash
ros2 service call /xx/set_data_ip lslidar_ls_driver/srv/DataIp "{data_ip: '192.168.1.200'}"   #Note: xx is the namespace, such as /ls1550
~~~

**Note: After setting, you need to modify the yaml file parameters and restart the driver.**



### Set lidar destination ip

source install/setup.bash

~~~bash
ros2 service call /xx/set_destination_ip lslidar_ls_driver/srv/DestinationIp "{destination_ip: '192.168.1.102'}"   #Note: xx is the namespace, such as /ls1550
~~~

**Note: After setting, you need to modify the Modify local IP address and restart the driver.**



### Set lidar data packet port

source install/setup.bash

~~~bash
ros2 service call /xx/set_data_port lslidar_ls_driver/srv/DataPort "{data_port: 2368}"  #range[1025,65535]   #Note: xx is the namespace, such as /ls1550
~~~

**Note: After setting, you need to modify the yaml file parameters and restart the driver.**



### Set lidar equipment packet port

source install/setup.bash

~~~bash
ros2 service call /xx/set_dev_port lslidar_ls_driver/srv/DevPort "{dev_port: 2369}"#range[1025,65535]   #Note: xx is the namespace, such as /ls1550
~~~

**Note: After setting, you need to modify the yaml file parameters and restart the driver.**



### Set lidar frame rate (lidar that supports modifying frame rate)

source install/setup.bash

~~~bash
ros2 service call /xx/set_frame_rate lslidar_ls_driver/srv/FrameRate "{frame_rate: 0}"   
#Note: xx is the namespace, such as /ls1550 
#0: Standard frame rate  
#25: A frame rate of 25% of the normal frame rate
#50: A frame rate of 50% of the normal frame rate
~~~



### Set lidar Time Synchronization Method

source install/setup.bash

**GPS Time Synchronization**

~~~bash
ros2 service call /xx/set_time_service lslidar_ls_driver/srv/TimeService "{time_service_mode: 'gps', ntp_ip: ''}" 
# Note: 'xx' with the namespace, for example, /ls1550
~~~

**PTP Time Synchronization**

~~~bash
# PTP L2
ros2 service call /xx/set_time_service lslidar_ls_driver/srv/TimeService "{time_service_mode: 'ptp_l2', ntp_ip: ''}" 
# Note: 'xx' with the namespace, for example, /ls1550

# PTP UDPV4
ros2 service call /xx/set_time_service lslidar_ls_driver/srv/TimeService "{time_service_mode: 'ptp_udpv4', ntp_ip: ''}" 
# Note: 'xx' with the namespace, for example, /ls1550
~~~

**NTP Time Synchronization**

~~~sh
ros2 service call /xx/set_time_service lslidar_ls_driver/srv/TimeService "{time_service_mode: 'ntp', ntp_ip: '192.168.1.102'}" 
# Note: 192.168.1.102 is the NTP IP address
~~~

**Note: After modification, please change the *time_synchronization* parameter to *true* in the YAML file.**



### Set lidar Standby Mode

source install/setup.bash

~~~bash
ros2 service call /xx/set_standby_mode lslidar_ls_driver/srv/StandbyMode "{standby_mode: 0}" 
# Note: 'xx' with the namespace, for example, /ls1550
# 0: Normal mode
# 1: Standby mode
~~~



### Set lidar Angle Distortion Correction

source install/setup.bash

~~~bash
ros2 service call /xx/set_angle_distortion_correction lslidar_ls_driver/srv/AngleDistortionCorrection "{angle_distortion_correction: 1}"
# Note: 'xx' with the namespace, for example, /ls1550
# 0: Disable
# 1: Enable
~~~

**Note: Please restart the driver program after modification.**



### Set lidar Invalid Data Transmission

source install/setup.bash

~~~bash
ros2 service call /xx/set_invalid_data lslidar_ls_driver/srv/InvalidData "{invalid_data: 1}"
# Note: 'xx' with the namespace, for example, /ls1550
# 0: Send
# 1: Do not send
~~~





## FAQ

Bug Report

Original version : LSLIDAR_LS128_ROS_V1.0.0_221012_ROS2

Modify:  original version

Date    : 2022-10-12

----------------



Update version : LSLIDAR_LS128_ROS_V1.0.1_221128_ROS2

Modify:  1. Upgrade the fpga version, and modify pointcloud calculation formula

​               2.Added support for ros2 humble/ros2 dashing / ros2 eloquent

Date    : 2022-11-28

-------------------



Update version : LSLIDAR_LS128_ROS_V1.0.2_230301_ROS2

Modify:  1. Added support for LS128S2 lidar

​                2.Change the fpga protocol and modify the overlapped frame judgment condition

Date    : 2023-03-01

----------------



Update version : LSLIDAR_LS128_ROS_V1.0.3_230313_ROS2

Modify:  1. Pre-compute sine and cosine value to reduce CPU consumption

​                2. Add packet loss detection function

Date    : 2023-03-13

Date    : 2023-03-01

----------------



Update version : LSLIDAR_LS1550_ROS_V1.0.4_240308_ROS2

Modify:  1.Compatible with MS06 lidar

​               2.Add lidar service settings

Date    : 2024-03-08

----



Update version : LSLIDAR_LS1550_V1.0.5_240808_ROS2

Modify: 1.Optimize code structure.

​			  2.Add lidar fault code information publication

​			  3.Merge message and service function packages.

​			  4.Support lidar angle distortion correction parsing.

​			  5.Add new features for setting and modifying lidar time source, standby mode, angle distortion correction, and invalid data transmission service.

Date    : 2024-08-08

----



Update version : LSLIDAR_LS1550_V1.0.6_240910_ROS2

Modify: 1.Updated the memory allocation method for point cloud data to avoid compatibility issues with boost.

Date    : 2024-09-10

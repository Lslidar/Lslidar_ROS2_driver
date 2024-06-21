# LSLIDAR_CHX1_V1.0.3_240621_ROS2

## 1.Introduction
​		LSLIDAR_CHX1_V1.0.3_240621_ROS2 is the lidar ros driver in linux environment, which is suitable for CX1S3,CX6S3,CH16X1,CH64W,CB64S1-A,CH128X1,CH128S1,CX126S3,CX128S2,CH256 lidar. The program has  tested under ubuntu18.04 ros dashing , ubuntu18.04 ros eloquent,ubuntu 20.04 ros foxy , ubuntu 20.04 ros galactic and ubuntu22.04 ros humble.

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
~~~



### 3.2 Run

run with single ch128s1 lidar:

~~~bash
source install/setup.bash
ros2 launch lslidar_ch_driver lslidar_ch_launch.py
ros2 launch lslidar_ch_driver lslidar_ch_rviz_launch.py		# Load RVIZ2
~~~

run with double ch128s1 lidar:

~~~bash
source install/setup.bash
ros2 launch lslidar_ch_driver lslidar_double_launch.py 
~~~



## 4. Introduction to parameters

The content of the lslidar_ch.yaml file is as follows, and the meaning of each parameter is shown in the notes.

~~~bash
/ch16x1/lslidar_driver_node:
  ros__parameters:
  	lidar_type: CX128S2             # lidar type  CX1S3/CX6S3/CH16X1/CH64W/CB64S1-A/CH128X1/CH128S1/CX126S3/CX128S2/CH256
    device_ip: 192.168.1.200        # lidar ip
    msop_port: 2368                 # Main data Stream Output Protocol packet port
    difop_port: 2369                # Device Information Output Protocol packet port
    add_multicast: false            # Whether to add multicast
    group_ip: 224.1.1.2             # multicast ip
    use_time_service: true          # Whether gps time synchronization
    min_range: 0.3                  # Unit: m. The minimum value of the lidar blind area, points smaller than this value are filtered
    max_range: 200.0                # Unit: m. The maximum value of the lidar blind area, points smaller than this value are filtered
    packet_rate: 1409.0             # Number of packets played per second when playing pcap
    scan_start_angle: 3000          # The minimum angle for radar scanning, points smaller than this value will be driven to filter. Unit: 0.01 °   64 lidar changed to 0
    scan_end_angle: 15000           # The maximum angle scanned by the radar, points greater than this value will be driven for filtering. Unit: 0.01 °   64 lidar changed to 18000
    frame_id: laser_link            # lidar point cloud coordinate system name
    topic_name: lslidar_point_cloud   # point cloud topic name, can be modified
    echo_num: 0                       # Only valid in double echo mode, 0 means release of all point clouds, 1 means release of the first echo point cloud, and 2 means release of the second echo point cloud
    publish_scan: false               # Whether to publish the scan
    channel_num: 16                   # laserscan line number
    horizontal_angle_resolution: 0.2  # 10Hz:0.2  20Hz:0.4 5Hz: 0.1
    packet_rate: 9826.0               # loading of PCAP packets, number of PCAP packets per second
    #pcap: /home/leishen/lidar.pcap   # Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar
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
  pcap: /home/leishen/lidar.pcap   // lidar.pcap is changed to the copied pcap file name
  ~~~



###  pcl point cloud type:

- Custom point cloud type, which references the definition in the file of

  /include/lslidar_ch_driver/lslidar_driver.h

  ~~~c++
  lslidar_ch_driver::PointXYZIRT,
                      (float, x, x)
                      (float, y, y)	
                      (float, z, z)
                      (float, intensity, intensity)
                      (std::uint8_t, ring, ring)
                      (float, time, time))
  ~~~




## FAQ

Bug Report

version : LSLIDAR_CHX1_V1.0.0_221116_ROS2

Modify:  original version

Date    : 2022-11-16

----



update version:LSLIDAR_CHX1_V1.0.1_230316_ROS2

Modify:  

Reduce CPU consumption,

Change the time of point to relative time,

New compatible cx128s2 radar.

Date    : 2023-03-16

----



update version:LSLIDAR_CHX1_V1.0.2_230419_ROS2

Modify:  

New compatible cx126s3 radar.

Date    : 2023-04-19

----



Update version : LSLIDAR_CHX1_V1.0.3_240621_ROS2

Modify:  

​			1. Merge feature packages

​			2.Optimize the code to reduce CPU usage.

​			3.Horizontal angle calculation for the new CX128S2 lidar.

​			4.Added compatibility with CX1S3, CX6S3, and CB64S1-A lidars.

Date    : 2024-06-21

----


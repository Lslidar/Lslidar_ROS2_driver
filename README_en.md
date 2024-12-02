# LSLIDAR_LS500_v1.0.0_231219_ROS2

## 1.Introduction
​		LSLIDAR_LS500_v1.0.0_231219_ROS2 is the lidar ros driver in linux environment, which is suitable for  LS500  lidar. The program has  tested under ubuntu18.04 ros dashing , ubuntu18.04 ros eloquent, ubuntu 20.04 ros foxy , ubuntu 20.04 ros galactic and ubuntu 22.04 ros humble .

## 2.Dependencies

1.ros

To run lidar driver in ROS environment, ROS related libraries need to be installed.

**Ubuntu 20.04**: ros-foxy-desktop-full

**Ubuntu 20.04**: ros-galactic-desktop-full

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
ros2 launch lslidar_driver lslidar_ls500_launch.py
~~~

run with double lidar:

~~~bash
ros2 launch lslidar_driver lslidar_ls500_double_launch.py
~~~

## 4. Introduction to parameters

The content of the lslidar_ls500.launch file is as follows, and the meaning of each parameter is shown in the notes.

~~~bash
/ls500/lslidar_driver_node:
  ros__parameters:
    device_ip: 192.168.1.200          #lidar ip
    msop_port: 2368                   # Main data Stream Output Protocol packet port
    difop_port: 2369                  # Device Information Output Protocol packet port
    frame_id: laser_link              # Coordinate system ID
    is_add_frame_: true		          # Whether to stacked frame
    add_multicast: false              # Whether to add multicast
    group_ip: 224.1.1.2               #multicast ip
    time_synchronization: false       # Whether gps time synchronization
    min_range: 0.3                    # Minimum measuring distance of lidar
    max_range: 500.0                  # Maximum measuring distance of lidar
    scan_start_angle: -60             #Scan start angle, range - 60 ° to 60 °
    scan_end_angle: 60                #Scan end angle,   range - 60 ° to 60 °
    topic_name: lslidar_point_cloud   # name of point cloud topic
    Vertical_angle_compensation: -40.5 # Vertical_angle_compensation
    packet_rate: 29815.0              # PCAP packets per second
    #pcap: /home/ls/Downloads/Ls500_60.pcap            #Uncomment to read the data from the pcap file, and add the comment to read the data from the radar
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

Drive to publish the total number of lidar packet loss in the form of topic with the name of "packet_loss" and the message type of std_ msgs::msg::Int32。



## FAQ

Bug Report

Original version :LSLIDAR_LS500_v1.0.0_231219_ROS2

Modify:  original version

Date    : 2023-12-19


Original version :LSLIDAR_LS500_v1.0.0_240308_ROS2

Modify:  Add vertical angle compensation

Date    : 2024-03-08


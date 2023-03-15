# LSLIDAR_CX_V4.1.0_221227_ROS2

## 1.Introduction
​		LSLIDAR_CX_V4.1.0_221227_ROS2 is the lidar ros driver in linux environment, which is suitable for ms-c16/c1/c8/c16/c32 lidar. The program has  tested under ubuntu18.04 ros dashing , ubuntu18.04 ros eloquent ,ubuntu 20.04 ros foxy,ubuntu 20.04 ros galactic  and ubuntu22.04 ros humble.

## 2.Dependencies

1.ros

To run lidar driver in ROS environment, ROS related libraries need to be installed.

**Ubuntu 20.04**: ros-foxy-desktop-full

**Ubuntu 20.04**: ros-galactic-desktop-full

**Ubuntu 18.04**: ros-dashing-desktop-full

**Ubuntu 18.04**: ros-eloquent-desktop-full

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

run with single c32 lidar:

~~~bash
ros2 launch lslidar_driver lslidar_c32_launch.py
~~~

run with double c32 lidar:

~~~bash
ros2 launch lslidar_driver lslidar_c32_double_launch.py
~~~



run with single c16 lidar:

~~~bash
ros2 launch lslidar_driver lslidar_c16_launch.py
~~~

run with double c16 lidar:

~~~bash
ros2 launch lslidar_driver lslidar_c16_double_launch.py
~~~



run with single c8 lidar:

~~~bash
ros2 launch lslidar_driver lslidar_c8_launch.py
~~~

run with double c8 lidar:

~~~bash
ros2 launch lslidar_driver lslidar_c8_double_launch.py
~~~



run with single c1 lidar:

~~~bash
ros2 launch lslidar_driver lslidar_c1_launch.py
~~~

run with double c1 lidar:

~~~bash
ros2 launch lslidar_driver lslidar_c1_double_launch.py
~~~



## 4. Introduction to parameters

The content of the lslidar_c32.yaml file is as follows, and the meaning of each parameter is shown in the notes.

~~~bash
/c32/lslidar_driver_node:
  ros__parameters:
    packet_rate: 1695.0             #Number of packets played per second when playing pcap
    device_ip: 192.168.1.200        #lidar ip
    msop_port: 2368                 #Main data Stream Output Protocol packet port
    difop_port: 2369                #Device Information Output Protocol packet port
    pcl_type: false                 #pointcloud type，false: xyzirt,true:xyzi
    lidar_type: c32                 # lidar type:  c1/c8/c16/c32
    c32_type: c32_32                #c32_32: lidar vertical angle is 30 degrees   c32_70: lidar vertical angle is 70 degrees(c32w)  c32_90: lidar vertical angle is 90 degrees(ch32r)
    add_multicast: false                    # Whether to add multicast
    group_ip: 224.1.1.2                     #multicast ip
    use_gps_ts: true                # Whether gps time synchronization
    min_range: 0.3                  #Unit: m. The minimum value of the lidar blind area, points smaller than this value are filtered
    max_range: 200.0                #Unit: m. The maximum value of the lidar blind area, points smaller than this value are filtered
    frame_id: laser_link            # lidar point cloud coordinate system name
    distance_unit: 0.4                      #lidar range resolution
    angle_disable_min: 0                    #lidar clipping angle start value ，unit:0.01°
    angle_disable_max: 0                    #lidar clipping angle end value ，unit:0.01°
    horizontal_angle_resolution: 0.2     #10Hz:0.2  20Hz:0.4 5Hz: 0.1
    scan_num: 16                            #laserscan line number
    topic_name: lslidar_point_cloud         #point cloud topic name, can be modified
    publish_scan: false                     #Whether to publish the scan
    pcl_type: false                         #pointcloud type，false: xyzirt,true:xyzi
    coordinate_opt: false                   #Default false. The zero degree angle of the lidar corresponds to the direction of the point cloud
    #pcap: /home/chris/Documents/leishen/1212bytes_c32/gps.pcap                        #Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar
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
ros2 service call /xx/time_service lslidar_msgs/srv/TimeService "{time_service_mode: 'gps',ntp_ip: ''}"   #Note: xx is the namespace, such as c1/c8/c16/c32
~~~

PTP：

~~~bash
ros2 service call /xx/time_service lslidar_msgs/srv/TimeService "{time_service_mode: 'ptp',ntp_ip: ''}"  #Note: xx is the namespace, such as c1/c8/c16/c32  
~~~

NTP：

~~~bash
ros2 service call /xx/time_service lslidar_msgs/srv/TimeService "{time_service_mode: 'ntp',ntp_ip: '192.168.1.102'}"   #Note: xx is the namespace, such as c1/c8/c16/c32
~~~



### lidar power on/off(lidar still rotates,  only send equipment packets):

source install/setup.bash

power on：

~~~bash
ros2 service call /xx/lslidar_control lslidar_msgs/srv/LslidarControl "{laser_control: 1}"   #Note: xx is the namespace, such as c1/c8/c16/c32
~~~

power off：

~~~bash
ros2 service call /xx/lslidar_control lslidar_msgs/srv/LslidarControl "{laser_control: 0}"   #Note: xx is the namespace, such as c1/c8/c16/c32
~~~



### lidar rotates/stops rotating (motor stops rotating)：

source install/setup.bash

转动：

~~~bash
ros2 service call /xx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: 1}"   #Note: xx is the namespace, such as c1/c8/c16/c32
~~~

停止转动：

~~~bash
ros2 service call /xx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: 0}"   #Note: xx is the namespace, such as c1/c8/c16/c32
~~~



### Set lidar speed:

source install/setup.bash

可选频率  5Hz/10Hz/20Hz

~~~bash
ros2 service call /xx/set_motor_speed lslidar_msgs/srv/MotorSpeed "{motor_speed: 20}"   #Note: xx is the namespace, such as c1/c8/c16/c32
~~~



### Set lidar data packet port

source install/setup.bash

~~~bash
ros2 service call /xx/set_data_port lslidar_msgs/srv/DataPort "{data_port: 2368}"  #range[1025,65535]   #Note: xx is the namespace, such as c1/c8/c16/c32
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



### Set lidar equipment packet port

source install/setup.bash

~~~bash
ros2 service call /xx/set_dev_port lslidar_msgs/srv/DevPort "{dev_port: 2369}"#range[1025,65535]   #Note: xx is the namespace, such as c1/c8/c16/c32
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



### Set lidar ip

source install/setup.bash

~~~bash
ros2 service call /xx/set_data_ip lslidar_msgs/srv/DataIp "{data_ip: "192.168.1.200"}"   #Note: xx is the namespace, such as c1/c8/c16/c32
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**



### Set lidar destination ip

source install/setup.bash

~~~bash
ros2 service call /xx/set_destination_ip lslidar_msgs/srv/DestinationIp "{destination_ip: "192.168.1.102"}"   #Note: xx is the namespace, such as c1/c8/c16/c32
~~~

**Note: After setting, you need to modify the launch file parameters and restart the driver.**





## FAQ

Bug Report

version : LSLIDAR_CX_V4.0.0_221031_ROS2

Modify:  original version

Date    : 2022-10-31

-----------------

version : LSLIDAR_CX_V4.1.0_221227_ROS2

Modify:

1.  Scan topic adds strength information
2. fpga upgrade, C32 90 degree modification of calculation formula
3. ROS driver adds the function of modifying time service mode
4. New function to modify lidar configuration
5. Added support for ros2 dashing, ros2 eloquent, and ros2 humble
5. Fixed the ntp timing resolution problem.

Date    : 2022-12-27

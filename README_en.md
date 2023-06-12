# LSLIDAR_CH120_V1.0.2_230302_ROS2

## 1.Introduction
​		LSLIDAR_CH120_V1.0.2_230302_ROS2 is the lidar ros driver in linux environment, which is suitable for ch120 lidar. The program has  tested under ubuntu 20.04 ros foxy , ubuntu 20.04 ros galactic and ubuntu22.04 ros humble.

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
source install/setup.bash
~~~

### 3.2 Run

run with single  lidar:

~~~bash
ros2 launch lslidar_driver lslidar_ch120_launch.py
~~~

run with double  lidar:

~~~bash
ros2 launch lslidar_driver lslidar_ch120_double_launch.py
~~~





## 4. Introduction to parameters

The content of the lslidar_ch120.yaml file is as follows, and the meaning of each parameter is shown in the notes.

~~~bash
/ch120/lslidar_driver_node:
  ros__parameters:
    device_ip: 192.168.1.200        #lidar ip
    lidar_type: ch120                 # lidar type: ch120
    msop_port: 2368                 #Main data Stream Output Protocol packet port
    difop_port: 2369                #Device Information Output Protocol packet port
    pcl_type: false                 #pointcloud type，false: xyzirt,true:xyzi
    add_multicast: false                    # Whether to add multicast
    group_ip: 224.1.1.2                     #multicast ip
    frame_id: laser_link            # lidar point cloud coordinate system name
    min_range: 0.3                  #Unit: m. The minimum value of the lidar blind area, points smaller than this value are filtered
    max_range: 500.0                #Unit: m. The maximum value of the lidar blind area, points smaller than this value are filtered
    packet_rate: 2525.0             #Number of packets played per second when playing pcap
    angle_disable_min: 0                    #lidar clipping angle start value ，range [0,180]
    angle_disable_max: 0                    #lidar clipping angle end value ，range [0,180]
    topic_name: lslidar_point_cloud         #point cloud topic name, can be modified
    horizontal_angle_resolution: 0.36     #10Hz:0.36  20Hz:0.72 5Hz: 0.18
    use_time_service: true                # Whether gps time synchronization
    echo_num: 0                          #Only valid in double echo mode, 0 means release of all point clouds, 1 means release of the first echo point cloud, and 2 means release of the second echo point cloud
    scan_num: 16                            #laserscan line number
    publish_scan: false                     #Whether to publish the scan
    #pcap: /home/chris/Documents/leishen/gps.pcap                        #Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar
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

## FAQ

Bug Report

Original version :LSLIDAR_CH120_V1.0.2_230302_ROS2

Modify:  original version

Date    : 2023-03-02

---------------


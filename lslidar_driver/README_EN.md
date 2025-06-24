## Instruction Manual for LSLIDAR_ROS2_V5.0.9_250305

### 1.Project Introduction
​		LSLIDAR_ROS2_V5.0.9_250305 is a ROS2 driver for LiDAR in Linux environments. The program has been successfully tested on Ubuntu 18.04 with ROS2 Dashing, Ubuntu 18.04 ROS2 with Eloquent, Ubuntu 20.04 with ROS2 Foxy, Ubuntu 20.04 with ROS2 Galactic, Ubuntu 22.04 with ROS2 Humble, and Ubuntu 24.04 with ROS2 Jazzy.

#### 1.1  Supported Lidar Models

```crystal
# Mechanical Lidars
N301 5.5
C16 C32	# 3.0, 4.0, 5.0 Lidars
C1 C1P C4 C8 C8F CKM8 MSC16 C16_domestic C32W C32WB C32WN C32WP CH32R CH32RN
```

~~~elixir
# # 905 Hybrid Solid-State Lidars
CX1S3 CX6S3 CH16X1 CH32A CH64W CB64S1_A CX126S3 CH128X1 CH128S1 CX128S2 CH256
~~~

```apl
# 1550 Hybrid Solid-State Lidars
LS25D    LS128S1  LS128S2  LS128S3  LS144S3 LS180S1  LS180S2  LS180S3
LS320S2  LS320S3  LS400S1  LS400S2  LS400S3 MS06
```



### 2.Dependencies

#### 2.1 ROS

+ Ubuntu 18.04  -  ROS2 Dashing desktop
+ Ubuntu 18.04  -  ROS2 Eloquent desktop

+ Ubuntu 20.04  -  ROS2 Foxy desktop
+ Ubuntu 20.04  -  ROS2 Galactic desktop
+ Ubuntu 22.04  -  ROS2 Humble desktop
+ Ubuntu 24.04  -  ROS2 Jazzy desktop
+ Ubuntu 24.04  -  ROS2 Rolling desktop

#### 2.2 ROS Dependencies 

```bash
sudo apt-get install ros-$ROS_DISTRO-pcl-conversions ros-$ROS_DISTRO-builtin-interfaces ros-$ROS_DISTRO-rosidl-default-generators 
```

#### 3.2  Other Dependencies

~~~bash
sudo apt-get install libpcl-dev libpcap-dev libyaml-cpp-dev libboost${BOOST_VERSION}-dev
~~~



## 3.Compilation and Execution

#### 3.1 Compilation

~~~bash
# Create a workspace and its src directory. lslidar_ws is the workspace name and can be customized.
mkdir -p ~/lidar_ws/src
# Unzip the driver package and place it in the ~/lidar_ws/src directory.
# Return to the workspace.
cd ~/lidar_ws
# Compile the project and refresh the terminal environment.
colcon build
source install/setup.bash
~~~

#### 3.2 Execution

To run a single lidar:

~~~bash
ros2 launch lslidar_driver lslidar_cx_launch.py # Mechanical LiDAR

ros2 launch lslidar_driver lslidar_ch_launch.py # 905 LiDAR

ros2 launch lslidar_driver lslidar_ls_launch.py # 1550 LiDAR
~~~

To run multiple lidars:

~~~bash
# Customize the launch file according to the actual situation
ros2 launch lslidar_driver lslidar_double_launch.py
~~~



## 4.Parameter Introduction

The content of the **lslidar_cx.yaml** file is as follows. More relevant files can be found in the `lslidar_driver/config` folder.。

For the meaning of each parameter, refer to the comments or consult **`Technical Support`**.。

~~~yaml
cx:                     # Ensure namespace matches corresponding launch files
  lslidar_driver_node:
    ros__parameters:
      lidar_type: "CX"            # LiDAR model
      device_ip: "192.168.1.200"  # LiDAR IP address
      msop_port: 2368             # Destination data port
      difop_port: 2369            # Destination device port
      use_time_service: false     # Enable time synchronization (GPS/PTP/NTP)
      packet_rate: 1695.0         # PCAP playback rate (for offline parsing)
      add_multicast: false        # Enable multicast
      group_ip: "224.1.1.2"       # Multicast IP address

      # Point cloud processing
      pcl_type: false             # Point cloud type format: true=XYZI
      frame_id: "laser_link"      # Coordinate frame name
      pointcloud_topic: "lslidar_point_cloud"  # Point cloud topic
      min_range: 0.15             # Minimum valid range (meters)
      max_range: 200.0            # Maximum valid range (meters)
      angle_disable_min: 0        # Minimum crop angle (unit: 0.01°) Fill in integers
      angle_disable_max: 0        # Maximum crop angle (unit: 0.01°) Fill in integers
      horizontal_angle_resolution: 0.18  # Laserscan angular resolution (5Hz:0.09 20Hz:0.36)
      publish_scan: false         # LaserScan output control
      scan_num: 8                 # LaserScan channel ID
      filter_angle_file: "config/filter_angle.yaml"  # Angle filter config path

      # Coordinate transformation
      is_pretreatment: false      # Enable point cloud preprocessing
      x_offset: 0.0               # X-axis translation (meters)
      y_offset: 0.0               # Y-axis translation (meters)
      z_offset: 0.0               # Z-axis translation (meters)
      roll: 0.0                   # X-axis rotation (radians)
      pitch: 0.0                  # Y-axis rotation (radians)
      yaw: 0.0                    # Z-axis rotation (radians)

      # pcap: "xxx.pcap"          # PCAP file path (for offline parsing)
~~~

### Explanation of Main Parameters：

- **lidar_type**

  Lidar type. For the 905 series, specify the **`specific model`**. For mechanical lidars, use **`CX`**, and for the 1550 series, use **`LS`**.

- **device_ip**

  The IP address of the lidar device (note that it is not the destination IP address of the lidar). You can check it using `wireshark` or `tcpdump`.

- **msop_port**

  The destination data port of the lidar (source port: 2369). Check it with `wireshark` or `tcpdump`.

- **difop_port**

  The destination device port of the lidar (source port: 2368). Check it using `wireshark` or `tcpdump`.

- **use_time_service**

  Time synchronization function (ensure the stability of the time source;  otherwise, it will affect the time continuity of the point cloud).

  - true: Use lidar time (GPS, NTP, PTP)
  - false: Use system time

- **packet_rate**

  Used when playing `PCAP` data packets offline. It refers to  the number of packets read per second. You can view the number of  packets per second of the lidar from the `I/O Chart` in `wireshark` statistics.

- **frame_id**

  The name of the coordinate system for the point cloud data published by  the program. Coordinate system transformation can be achieved through `tf`.

- **pointcloud_topic**

  The topic name for the point cloud data published by the program.

- **min_range  max_range**

  - **`min_range`**: Starting distance of the point cloud.

  - **`max_range`**: Ending distance of the point cloud.

    *This setting is a software filter that will remove points outside this range. Unit: 1m*

- **scan_start_angle  scan_end_angle  angle_disable_min  angle_disable_max**

  - **`scan_start_angle`**: Starting horizontal angle of the point cloud.

    **`scan_end_angle`**: Ending horizontal angle of the point cloud.

    **`angle_disable_min`**: Starting horizontal angle cropping for the point cloud.

    **`angle_disable_max`**: Ending horizontal angle cropping for the point cloud.

    *This setting is a software filter that will remove points outside this angular range. Unit: 0.01°. Enter an integer.*

- **is_pretreatment**

  Point cloud pre - processing. When this value is `true`, the point cloud data will be transformed in spatial position and orientation according to the following parameters.

  - **`x_offset`**: Visualized as displacement along the red X-axis in rviz2 (unit: m)  
  - **`y_offset`**: Visualized as displacement along the green Y-axis in rviz2 (unit: m)  
  
  - **`z_offset`**: Visualized as displacement along the blue Z-axis in rviz2 (unit: m)  
  
  - **`roll`**:    Visualized as rotation about the red X-axis in rviz2 (unit: rad)  

  - **`pitch`**:  Visualized as rotation about the green Y-axis in rviz2 (unit: rad)  
  
  -  **`yaw`**:     Visualized as rotation about the blue Z-axis in rviz2 (unit: rad)
  
    
  
  

### Explanation of Special Parameters：

#### The following functions are only supported by specific series of lidars.

- **pcl_type**

  Point cloud data type. When this value is `true`, the point cloud data is published in the x, y, z, i format (coordinates and intensity).

- **filter_angle_file**

  - **`disable_min`**: Point cloud data with an angle smaller than this will be filtered out.
  - **`disable_max`**: Point cloud data with an angle smaller than this will be filtered out.

  Custom angle cropping for individual scan lines is allowed. You can set the  cropping angle range for each scan line separately. Modify the **`lslidar_driver/param/filter_angle.yaml`** file according to your actual needs.

- **publish_scan**

  Publish `LaserScan` data. When this value is `true`, `LaserScan` data will be published.

- **scan_num**

  Specify which scan line is used to publish `LaserScan` data. The valid line numbers depend on the number of scan lines of the lidar.

- **echo_mode**

  Echo mode. 0: Publish all point clouds 1: Publish the first echo point cloud 2: Publish the second echo point cloud(Effective in dual echo mode).

- **packet_loss**

  Packet loss detection. Once enabled, the driver will publish the total number of lidar packet losses as a topic named **`packet_loss`**, with the message type **`std_msgs::msg::Int64`**.

  

​		

### Multicast Mode:

- Set the destination IP of the lidar to a multicast network segment.

- Modify the following parameters in the launch file:

  ~~~yaml
  add_multicast: false     # Whether to enable multicast
  group_ip: "224.1.1.2"    # Multicast IP address
  ~~~

- Run the following commands to add the computer to the multicast group (replace `enp2s0` with the network card name of your computer, which can be checked using `ifconfig`):

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### Offline PCAP Playback Mode:

- Get the path of the recorded PCAP file.

- Modify the following parameters in the YAML file.

  ~~~yaml
  # Path of the pcap package. Uncomment this when loading the pcap package.
  pcap: "xxx.pcap"  
  ~~~



### PCL Point Cloud Type：

- he point cloud published by the driver is a custom point cloud type. For the definition, refer to **`lslidar_driver/include/lslidar_pointcloud.hpp`**.

  ~~~c++
  struct PointXYZIRT {
      PCL_ADD_POINT4D;     // Coordinates x, y, z
      PCL_ADD_INTENSITY;   // intensity
      std::uint16_t ring;  // Line number
      float time;          // time
  
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
  ~~~



### Device Information:

- The lidar driver publishes the device information in the form of a topic. The topic name is `lslidar_device_info`, and the message type is `lslidar_msgs::msg::LslidarInformation`. For the definition, refer to **`lslidar_msgs/LslidarInformation.msg`**.

  

### Fault Codes：

- The LS (1550) series lidar driver publishes the fault codes in the form of a topic. The topic name is `lslidar_fault_code`, and the message type is `std_msgs::msg::String`.

  *For the meaning of each fault code, please contact the `technical support`.*



### Point Cloud Time：

- The lidar driver publishes the point cloud time in the form of a topic. The topic name is `time_topic`, and the message type is `std_msgs::msg::Float64`.





## 5.Lidar Function Settings and Control

#### **All setting functions can be implemented by ROS services. Some functions are only supported by specific series of lidars.**



### Setting the Lidar Network Configuration：

~~~bash
# Open a new terminal. cx is the namespace.
source install/setup.bash

# Configure the lidar using explicit parameters.
ros2 service call /cx/network_setup lslidar_msgs/srv/IpAndPort "{lidar_ip: '192.168.1.200',destination_ip: '192.168.1.102',data_port: '2368',dev_port: '2369'}

# Configure the lidar using the parameters in the configuration file (network_setup.yaml).
ros2 service call /cx/network_setup lslidar_msgs/srv/IpAndPort "{lidar_ip: '', destination_ip: '', data_port: 0, dev_port: 0}"
~~~

#### ***Restart the driver after making changes.***





### Lidar Timing Mode：

~~~bash
# Open a new terminal. cx is the namespace.
source install/setup.bash

# 0: GPS   1: PTP L2   2: NTP   3: PTP UDPv4   4: E2E L2   5: E2E UDPv4
ros2 service call /cx/time_mode lslidar_msgs/srv/TimeMode "{time_mode: '0', ntp_ip: ''}"

# When setting NTP timing, you need to fill in the NTP IP.
ros2 service call /cx/time_mode lslidar_msgs/srv/TimeMode "{time_mode: '2', ntp_ip: '192.168.1.102'}"
~~~





### Lidar Power On/Off (Mechanical):

~~~bash
# Open a new terminal. cx is the namespace.
source install/setup.bash
~~~

Power On：

~~~bash
ros2 service call /cx/power_control lslidar_msgs/srv/PowerControl "{power_control: '1'}"
~~~

Power Off：

~~~bash
ros2 service call /cx/power_control lslidar_msgs/srv/PowerControl "{power_control: '0'}"
~~~





### Lidar Rotation/Stop Rotation (Mechanical):

~~~bash
# Open a new terminal. cx is the namespace.
source install/setup.bash
~~~

Rotation：

~~~bash
ros2 service call /cx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: '1'}"
~~~

Stop Rotation：

~~~bash
ros2 service call /cx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: '0'}"
~~~





### Lidar Rotation Speed (Mechanical, 905 Series):

~~~bash
# Open a new terminal. cx is the namespace.
source install/setup.bash
# Optional frequency  5Hz/10Hz/20Hz
ros2 service call /cx/motor_speed lslidar_msgs/srv/MotorSpeed "{motor_speed: '20'}"
~~~





### Rain, Fog, and Dust Removal Level of the Lidar(Mechanical)：

~~~bash
# Open a new terminal. cx is the namespace.
source install/setup.bash
#Available levels: 0/1/2/3. The higher the number, the stronger the removal effect.
ros2 service call /cx/remove_rain_fog_dust lslidar_msgs/srv/RfdRemoval "{rfd_removal: '3'}"
~~~





### Tail Removal Level of the Lidar(Mechanical)：

~~~bash
# Open a new terminal. cx is the namespace.
source install/setup.bash
# Available levels range from 0-10. The higher the number, the stronger the removal effect.
# The maximum level for older lidar models is 4.
ros2 service call /cx/tail_remove lslidar_msgs/srv/TailRemoval "{tail_removal: '4'}"
~~~





### Angle Distortion Correction of the Lidar(1550 Series)：

~~~bash
# Open a new terminal. ls is the namespace.
source install/setup.bash
~~~

Turn off angle distortion correction:：

~~~bash
ros2 service call /ls/angle_distortion_correction lslidar_msgs/srv/AngleDistortionCorrection "{angle_distortion_correction: '0'}"
~~~

Turn on angle distortion correction:：

~~~bash
ros2 service call /ls/angle_distortion_correction lslidar_msgs/srv/AngleDistortionCorrection "{angle_distortion_correction: '1'}"
~~~

**Enabling this function can reduce CPU usage**

#### ***Restart the driver after making changes.***





### Lidar Frame Rate(1550 Series)：

~~~bash
# Open a new terminal. ls is the namespace.
source install/setup.bash
# 0: Normal frame rate;  1: 50% frame rate;  2: 25% frame rate
ros2 service call /ls/frame_rate lslidar_msgs/srv/FrameRate "{frame_rate: '1'}"	
# 50% frame rate, 5Hz
~~~





### Sending of Invalid Lidar Data(1550 Series)：

~~~bash
# Open a new terminal. ls is the namespace.
source install/setup.bash
~~~

Send invalid data:

~~~bash
ros2 service call /ls/invalid_data lslidar_msgs/srv/InvalidData "{invalid_data: '0'}"
~~~

Do not send invalid data：

~~~bash
ros2 service call /ls/invalid_data lslidar_msgs/srv/InvalidData "{invalid_data: '1'}"
~~~

***Not sending invalid data can reduce CPU usage, but it will cause discontinuity in point - cloud time.***





### Lidar Standby Mode(1550)：

~~~bash
# Open a new terminal. ls is the namespace.
source install/setup.bash
~~~

Normal mode：

~~~bash
ros2 service call /ls/standby_mode lslidar_msgs/srv/StandbyMode "{standby_mode: '0'}"
~~~

Standby mode：

~~~bash
ros2 service call /ls/standby_mode lslidar_msgs/srv/StandbyMode "{standby_mode: '1'}"
~~~







## FAQ

Bug Report

Original version : LSLIDAR_ROS2_V5.0.9_250305

Modify: original version

Date    : 2025-03-05

--------------------------------------------------------------------


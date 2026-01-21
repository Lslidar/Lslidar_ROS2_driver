# LSLIDAR_ROS2_V5.2.0_20260112使用说明

## 1.工程介绍
​		LSLIDAR_ROS2_V5.2.0_20260112为linux环境下雷达ROS2驱动，程序在Ubuntu 18.04 ROS2 Dashing 和 Ubuntu 18.04 ROS2 Eloquent 和 Ubuntu 20.04 ROS2 Foxy 和 Ubuntu 20.04 ROS2 Galactic 和 Ubuntu 22.04 ROS2 Humble 和 Ubuntu 24.04 ROS2 Jazzy 和 Ubuntu 24.04 ROS2 Rolling下测试通过。

#### 1.1 支持的雷达型号

~~~bash
# 单线激光雷达
M10 M10GPS M10P N10	N10Plus N301(1.6 1.7)
~~~

```crystal
# 机械式雷达
N301 5.5
C16 C32	#3.0 4.0 5.0雷达
C1 C1P C4 C8 C8F CKM8 MSC16 C16_domestic C32W C32WB C32WN C32WP CH32R CH32RN
```

~~~elixir
# 905混合固态雷达
CX1S3 CX6S3 CH16X1 CH32A CH64W CB64S1_A CX126S3 CH128X1 CH128S1 CX128S2 CH256 CH1W 
~~~

```apl
# 1550混合固态雷达
LS25D    LS128S1  LS180S1  LS400S1     
LS128S2  LS180S2  LS320S2  LS400S2  MS06
LS128S3  LS144S3  LS180S3  LS320S3  LS400S3
LSS4
```



### 2.依赖

#### 2.1 ROS

+ Ubuntu 18.04  -  ROS2 Dashing desktop
+ Ubuntu 18.04  -  ROS2 Eloquent desktop

+ Ubuntu 20.04  -  ROS2 Foxy desktop
+ Ubuntu 20.04  -  ROS2 Galactic desktop
+ Ubuntu 22.04  -  ROS2 Humble desktop
+ Ubuntu 24.04  -  ROS2 Jazzy desktop
+ Ubuntu 24.04  -  ROS2 Rolling desktop

#### 2.2 ROS依赖 

```bash
sudo apt-get install ros-$ROS_DISTRO-pcl-conversions ros-$ROS_DISTRO-builtin-interfaces ros-$ROS_DISTRO-rosidl-default-generators 
```

#### 3.2 其他依赖

~~~bash
sudo apt-get install libpcl-dev libpcap-dev libyaml-cpp-dev libboost${BOOST_VERSION}-dev
~~~



## 3.编译运行

#### 3.1 编译

~~~bash
#创建工作空间及src目录  lslidar_ws为工作空间名 可自定义
mkdir -p ~/lslidar_ws/src
#将驱动压缩包解压缩放到~/lslidar_ws/src 目录下
#返回工作空间
cd ~/lslidar_ws
#编译及刷新终端环境
colcon build
source install/setup.bash
~~~

#### 3.2 运行

运行单个雷达:

~~~bash
ros2 launch lslidar_driver lslidar_x10_launch.py # 单线雷达

ros2 launch lslidar_driver lslidar_cx_launch.py # 机械式雷达

ros2 launch lslidar_driver lslidar_ch_launch.py # 905雷达

ros2 launch lslidar_driver lslidar_ls_launch.py # 1550雷达
~~~

运行多个雷达：

~~~bash
# 可根据实际情况自定义launch文件
ros2 launch lslidar_driver lslidar_double_launch.py
~~~



## 4.参数介绍

**lslidar_cx.yaml**文件内容如下，更多文件在`lslidar_driver/config`文件夹下。

每个参数含义见注释说明或咨询**`技术支持`**。

~~~YAML
cx:  							  # 确保与对应launch文件中命名空间一致
  lslidar_driver_node:
    ros__parameters:
      lidar_type: "CX"            # 雷达型号
      device_ip: "192.168.1.200"  # 雷达IP地址
      msop_port: 2368             # 雷达目的数据端口
      difop_port: 2369            # 雷达目的设备端口
      packet_rate: 1695.0         # PCAP文件回放速率，离线解析PCAP数据时使用
      add_multicast: false        # 雷达是否开启组播
      group_ip: "224.1.1.2"       # 组播IP地址
      #pcap: "xxx.pcap"          # PCAP文件路径，离线解析PCAP数据时打开注释
      
      # 点云处理参数
      pcl_type: false             # 点云类型  true: xyzi
      frame_id: "laser_link"      # 坐标系名称
      pointcloud_topic: "lslidar_point_cloud"  # 点云话题名
      use_time_service: false     # 雷达是否使用授时(GPS PTP NTP)
      use_first_point_time: false # true: 使用每帧第一个点的时间做为点云时间
      use_absolute_time: false    # 点时间格式 true: 使用绝对时间，与点云头文件中保持一致
      min_range: 0.15             # 雷达扫描最小距离 小于该值的点将被过滤(m)
      max_range: 200.0            # 雷达扫描最大距离 大于该值的点将被过滤(m)
      angle_disable_min: 0        # 雷达扫描最小裁剪角度 填整数 单位: 0.01°
      angle_disable_max: 0        # 雷达扫描最大裁剪角度 填整数 单位: 0.01°
      horizontal_angle_resolution: 0.18  # Laserscan 水平角分辨率  5Hz:0.09 20Hz:0.36 
      publish_scan: false         # LaserScan输出控制
      scan_num: 0                 # LaserScan线号
      filter_angle_file: "config/filter_angle.yaml"  # 角度滤波文件路径

      # 点云坐标系转换参数
      is_pretreatment: false      # 预处理开关 仅对点云生效
      x_offset: 0.0               # X轴平移(m) 
      y_offset: 0.0               # Y轴平移(m)
      z_offset: 0.0               # Z轴平移(m)
      roll: 0.0                   # X轴旋转(rad)
      pitch: 0.0                  # Y轴旋转(rad)
      yaw: 0.0                    # Z轴旋转(rad)
      
      is_MatrixTransformation: false  # 预处理开关 true 使用以下两个(4*4)矩阵进行转换(若只转换一次，可将其中一个删除)，false不使用
      transform_main:
        [
          0.3514713644981384,
          0.3227383494377136,
          0.878810465335846,
          1.250789999961853,
          -0.17116132378578186,
          0.9450311660766602,
          -0.27860331535339355,
          -0.3601331412792206,
          -0.9204192757606506,
          -0.05249723792076111,
          0.38739171624183655,
          1.9063990116119385,
          0.0,
          0.0,
          0.0,
          1.0,
        ]
      transform_imu:
        [
          0.999537,
          -0.030429,
          0.00027,
          -1.047623,
          0.030413,
          0.999231,
          0.024782,
          5.966497,
          -0.001024,
          -0.024763,
          0.999693,
          0.638128,
          0.0,
          0.0,
          0.0,
          1.0,
        ]
~~~

### 主要参数说明：

- **lidar_type**

  雷达类型，905系列需指定**`具体型号`**，机械式为**`CX`**，1550系列为**`LS`**

- **lidar_model**

  雷达型号，激光雷达具体型号，如：M10, CX126S3, LSS3, LSS4等。

- **device_ip**

  雷达设备IP地址(注意不是雷达目的IP地址)，可使用`wireshark`或`tcpdump`查看。

- **msop_port**

  雷达目的数据端口(源端口: 2369)，可使用`wireshark`或`tcpdump`查看。

- **difop_port**

  雷达目的设备端口(源端口: 2368)，可使用`wireshark`或`tcpdump`查看。

- **use_time_service**

  授时功能(请确保授时源稳定，否则影响点云时间连续性)。

  - true:  使用雷达时间(GPS, NTP, PTP)
  - false: 使用系统时间

- **use_first_point_time**

  点云时间

  - true:  使用每帧第一个点的时间做为点云时间
  - false: 使用每帧最后一个点的时间做为点云时间

- **use_absolute_time**

  点的时间模式(注意此值要与**`lslidar_pointcloud.hpp`**文件中**`USE_ABSOLUTE_TIME`**保持一致)

  - true:  点的时间使用绝对时间格式
  - false: 点的时间使用相对时间格式

- **packet_rate**

  PCAP文件回放速率，每秒读取数据包个数。雷达每秒包数可使用`wireshark`统计下的`I/O图表`查看

- **frame_id**

  程序发布点云数据坐标系名称，可通过`tf`实现坐标系间的变换

- **pointcloud_topic**

  程序发布点云数据话题名称

- **min_range  max_range**

  - **`min_range`**： 点云起始距离

  - **`max_range`**：点云结束距离

    *此设置为软件屏蔽，会将区域外的点过滤。 单位：1m*

- **scan_start_angle  scan_end_angle  angle_disable_min  angle_disable_max**

  - **`scan_start_angle`**: 点云起始水平角度

  - **`scan_end_angle`**:  点云结束水平角度

  - **`angle_disable_min`**: 点云起始水平角度裁剪

  - **`angle_disable_max`**: 点云结束水平角度裁剪

    *此设置为软件屏蔽，会将区域外的点过滤。 单位：0.01° 填整数*

- **is_pretreatment**

  - 点云预处理，此值为ture时点云数据根据以下参数进行空间位置和方向的变换。
  - **`x_offset`**: rviz2 中表现为围绕红色的 x 轴偏移量 单位: m 
  - **`y_offset`**: rviz2 中表现为围绕绿色的 y 轴偏移量 单位: m
  - **`z_offset`**: rviz2 中表现为围绕蓝色的 z 轴偏移量 单位: m
  - **`roll`**:    rviz2 中表现为围绕红色的 x 轴旋转   单位: rad
  - **`pitch`**:  rviz2 中表现为围绕绿色的 y 轴旋转   单位: rad
  - **`yaw`**:      rviz 中表现为围绕蓝色的 z 轴旋转   单位: rad
  
- is_MatrixTransformation

  - 点云预处理，此值为ture时点云数据根据使用以下两个矩阵参数进行空间位置和方向的变换。 若只转换一次，可将其中一个删除
  - transform_main：变换的矩阵参数1， 4*4矩阵
  - transform_imu：变换的矩阵参数2， 4*4矩阵


### 特殊参数说明：

#### 以下功能仅对特定系列雷达支持

- **serial_port**

  激光雷达连接的串口名称（例如 `/dev/ttyUSB0` 或 `/dev/ttyACM0`），**使用串口雷达时，请填写正确的串口名称，并确保已授予该串口的读写权限**。

- **use_high_precision**

  高精度模式，laserscan数据启用高精度模式，角度分辨率减小10倍，数据量增加10倍，提高精度。

- **publish_multiecholaserscan**

  N10Plus雷达是否发布`sensor_msgs::MultiEchoLaserScan`数据，此值为true时发布数据。

- **enable_noise_filter**

  孤立噪点滤波开关，此值为true时将过滤孤立点(N10Plus不生效)。

- **pcl_type**

  点云数据类型，此值为true时使用 x y z i 类型发布点云数据(坐标，强度)。

- **filter_angle_file**

  - **`disable_min`**: 小于此角度的点云数据将被过滤掉
  - **`disable_max`**: 小于此角度的点云数据将被过滤掉

  单独线号自定义角度裁剪，允许对每条扫描线分别设置裁剪角度范围。可根据实际使用情况更改**`lslidar_driver/param/filter_angle.yaml`**文件。

- **publish_scan**

  发布 `LaserScan` 数据，此值为true时发布 `LaserScan` 数据。

- **scan_num**

  指定用于发布`LaserScan`数据的线号，线号范围根据雷达扫描线束而定。

- **echo_mode**

  回波模式，0:发布全部点云  1:发布第一次回波点云  2:发布第二次回波点云(双回波模式下生效)。

- **is_add_frame**

  叠帧发布点云信息，将连续两帧点云数据叠加后一起发布。

- **packet_loss**

  丢包检测，开启后驱动将雷达丢包总数以话题的形式发布，话题名字为**`packet_loss`**，消息类型为**`std_msgs::msg::Int64`**。

  

​		

### 组播模式：

- 将雷达目的IP设置为组播网段

- 修改yaml文件以下参数

  ~~~yaml
  add_multicast: false        # 雷达是否开启组播
  group_ip: "224.1.1.2"       # 组播IP地址
  ~~~

- 运行以下指令将电脑加入组内（将指令中的enp2s0替换为用户电脑的网卡名,可用ifconfig查看网卡名)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### 离线播放PCAP模式：

- 获取录制好的`PCAP`文件路径。

- 修改yaml文件以下参数

  ~~~yaml
  #pcap包路径，加载pcap包时打开此注释
  pcap: "xxx.pcap"          # PCAP文件路径，打开注释并填写对应文件路径
  ~~~



###  pcl点云类型：

- 驱动发布点云为自定义点云类型，定义参考**`lslidar_driver/include/lslidar_pointcloud.hpp`**

- 如果使用绝对时间，请取消 **`#define USE_ABSOLUTE_TIME`**的注释

  ~~~c++
  struct PointXYZIRT {
      PCL_ADD_POINT4D;      // x, y, z 和 data[4]
      PCL_ADD_INTENSITY;    // 强度
      std::uint16_t ring;   // 线号
      POINT_TIME_TYPE time; // 时间  POINT_TIME_TYPE -> float/double 
  
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
  ~~~



### 设备信息：

- 雷达驱动将设备信息以话题的形式发布，话题名字为`lslidar_device_info`，消息类型为`lslidar_msgs::msg::LslidarInformation`定义参考**`lslidar_msgs/LslidarInformation.msg`**



### 故障码：

- LS(1550)系列雷达驱动将故障码以话题的形式发布，话题名字为`lslidar_fault_code`，消息类型为`std_msgs::msg::String`

  *每位故障码含义请联系技术支持*



### 点云时间：

- 雷达驱动将点云时间以话题的形式发布，话题名字为`time_topic`，消息类型为`std_msgs::msg::Float64`





## 5.雷达功能设置与调控

#### **部分功能只对特定系列雷达支持。**

### 设置雷达网络配置：

~~~bash
#新开一个终端 cx为命名空间
source install/setup.bash

# 使用显示传参配置雷达
ros2 service call /cx/network_setup lslidar_msgs/srv/IpAndPort "{lidar_ip: '192.168.1.200',destination_ip: '192.168.1.102',data_port: '2368',dev_port: '2369'}

# 使用配置文件参数来配置雷达(network_setup.yaml)
ros2 service call /cx/network_setup lslidar_msgs/srv/IpAndPort "{lidar_ip: '', destination_ip: '', data_port: 0, dev_port: 0}"
~~~

#### ***更改后需重启驱动***





### 雷达授时方式：

~~~bash
#新开一个终端	cx为命名空间
source install/setup.bash

# 0: GPS   1: PTP L2   2: NTP   3: PTP UDPv4   4: E2E L2   5: E2E UDPv4
ros2 service call /cx/time_mode lslidar_msgs/srv/TimeMode "{time_mode: '0', ntp_ip: ''}"

# 设置NTP授时是需要填写ntp ip
ros2 service call /cx/time_mode lslidar_msgs/srv/TimeMode "{time_mode: '2', ntp_ip: '192.168.1.102'}"
~~~





### 雷达上下电(机械式)：

~~~bash
#新开一个终端 cx为命名空间
source install/setup.bash
~~~

上电：

~~~bash
ros2 service call /cx/power_control lslidar_msgs/srv/PowerControl "{power_control: '1'}"
~~~

下电：

~~~bash
ros2 service call /cx/power_control lslidar_msgs/srv/PowerControl "{power_control: '0'}"
~~~





### 雷达转动/停止转动(机械式)：

~~~bash
#新开一个终端 cx为命名空间
source install/setup.bash
~~~

转动：

~~~bash
ros2 service call /cx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: '1'}"
~~~

停止转动：

~~~bash
ros2 service call /cx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: '0'}"
~~~





### 雷达转速(机械式 905)：

~~~bash
#新开一个终端 cx为命名空间
source install/setup.bash
#可选频率  5Hz/10Hz/20Hz  (扩展30 40 50 60仅只有CX1S3可设置)
ros2 service call /cx/motor_speed lslidar_msgs/srv/MotorSpeed "{motor_speed: '20'}"
~~~





### 雷达去除雨雾尘等级(机械式)：

~~~bash
#新开一个终端 cx为命名空间
source install/setup.bash
#可选等级  0/1/2/3 ，0-3 数字越大，去除越强
ros2 service call /cx/remove_rain_fog_dust lslidar_msgs/srv/RfdRemoval "{rfd_removal: '3'}"
~~~





### 雷达去拖尾等级(机械式)：

~~~bash
#新开一个终端 cx为命名空间
source install/setup.bash
#可选等级 0-10 数字越大，去除越强
#旧版本雷达最大为4
ros2 service call /cx/tail_remove lslidar_msgs/srv/TailRemoval "{tail_removal: '4'}"
~~~





### 雷达角度畸变矫正(LSS3)：

**LSS4系列雷达暂无此功能，调用此服务将配置是否发送无效数据**

~~~bash
#新开一个终端 ls为命名空间
source install/setup.bash
~~~

关闭角度畸变矫正：

~~~bash
ros2 service call /ls/angle_distortion_correction lslidar_msgs/srv/AngleDistortionCorrection "{angle_distortion_correction: '0'}"
~~~

开启角度畸变矫正：

~~~bash
ros2 service call /ls/angle_distortion_correction lslidar_msgs/srv/AngleDistortionCorrection "{angle_distortion_correction: '1'}"
~~~

**开启此功能可减少CPU占用**

#### *更改后需重启驱动*





### 雷达帧率(1550)：

~~~bash
#新开一个终端 ls为命名空间
source install/setup.bash
# 0: 正常帧率    1: 50%帧率    2: 25%帧率
ros2 service call /ls/frame_rate lslidar_msgs/srv/FrameRate "{frame_rate: '1'}"	
# 50%帧率  5hz
~~~







### **雷达无效数据发送**(1550)：

LSS4系列雷达此功能请调用`angle_distortion_correction`服务

~~~bash
#新开一个终端 ls为命名空间
source install/setup.bash
~~~

发送无效数据：

~~~bash
ros2 service call /ls/invalid_data lslidar_msgs/srv/InvalidData "{invalid_data: '0'}"
~~~

不发送无效数据：

~~~bash
ros2 service call /ls/invalid_data lslidar_msgs/srv/InvalidData "{invalid_data: '1'}"
~~~

***不发送无效数据可以减少CPU占用，但会导致点云时间不连续***





### 雷达待机模式(1550)：

~~~bash
#新开一个终端 ls为命名空间
source install/setup.bash
~~~

正常模式：

~~~bash
ros2 service call /ls/standby_mode lslidar_msgs/srv/StandbyMode "{standby_mode: '0'}"
~~~

待机模式：

~~~bash
ros2 service call /ls/standby_mode lslidar_msgs/srv/StandbyMode "{standby_mode: '1'}"
~~~





### 单线雷达电机控制：

~~~bash
# N301暂不支持
# 新开一个终端 x10为命名空间
source install/setup.bash
~~~

转动：

~~~bash
ros2 topic pub -1 /x10/motor_control std_msgs/msg/Int8 "{data: '1'}"
~~~

停转并不发数据：

~~~bash
ros2 topic pub -1 /x10/motor_control std_msgs/msg/Int8 "{data: '0'}"
~~~







## FAQ

Bug Report

Original version : LSLIDAR_ROS2_V5.0.9_250305

Modify: original version

Date    : 2025-03-05

--------------------------------------------------------------------



Update version : LSLIDAR_ROS2_V5.1.0_250430

Modify: 

1. 新增兼容M10, M10GPS, M10P, N10, N10Plus, N301雷达
1. N10Plus雷达新增MultiEchoLaserScan消息
1. 新增laserscan高精度模式
1. 新增单线雷达电机控制功能
1. 新增兼容1550 LSS4系列雷达
1. 新增点云时间与点时间格式选择

Date    : 2025-04-30

--------------------------------------------------------------------

Update version : LSLIDAR_ROS2_V5.1.0_250507

Modify: 

1. 新增git 的管理

Date    : 2025-05-07

--------------------------------------------------------------------

Update version : LSLIDAR_ROS2_V5.1.1_250527

Modify: 

1. 新增矩阵参数，进行预处理

Date    : 2025-05-27

------

Update version : LSLIDAR_ROS2_V5.1.2_250808

Modify: 

1. 新增C_v3.0定制版本数据包长度为1212byte的兼容
2. 新增CH1W雷达型号的兼容

Date    : 2025-08-08

------

update version : LSLIDAR_ROS2_V5.1.3_250909

Modify: 

1. 扩展转速设置 (扩展到30 40 50 60仅只有CX1S3可设置)
1. CX1S3修改使用laserscan 点云跳动不正确的问题

Date    : 2025-09-09

------

update version : LSLIDAR_ROS2_V5.1.4_251104

Modify: 

1. 设置 UDP 接收缓存区 2M
1. 增加雷达判断是否使用雷达范围初始化 scan
1. 修改 m10/n10 系列雷达点云相反的问题
1. 修改cx和ch 发布LaserScan时候，  scan_msg_bak判断是否为空

Date    : 2025-11-04

------

update version : LSLIDAR_ROS2_V5.2.0_20260112

Modify: 

1. 增加定义的结构体 每个点云的RGB赋值
1. 修改LSS3驱动 兼容MS06

Date    : 2026-01-12

------


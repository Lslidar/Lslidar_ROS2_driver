# lslidar

## Description
The `lslidar package is a linux ROS2 driver for lslidar M10 ,M10_GPS,M10_P,M10_PLUS and N10.
The package is tested on Ubuntu 20.04 with ROS2 FOXY.

## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
colcon build
source install/setup.bash
ros2 launch lslidar_driver lslidar_launch.py
```
open new terminal
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 1\ 		(open radar)
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 0\ 		(close radar)





ros2 launch lslidar_driver lslidar_launch.py

```

Note that this launch file launches both the driver, which is the only launch file needed to be used.


## FAQ


## Bug Report

Prefer to open an issue. You can also send an E-mail to honghangli@lslidar.com




RERTION 




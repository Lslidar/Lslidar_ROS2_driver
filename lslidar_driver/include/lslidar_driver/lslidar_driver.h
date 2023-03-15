/******************************************************************************
 * This file is part of lslidar_cx driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/


#ifndef _LS_C16_DRIVER_H_
#define _LS_C16_DRIVER_H_

#define DEG_TO_RAD 0.017453293
#define RAD_TO_DEG 57.29577951

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
//#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
//#include <boost/shared_ptr.hpp>
//#include <boost/thread.hpp>
#include <thread>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
//#include <sensor_msgs/msg/laser_scan.h>
#include "input.h"
//#include <pcl_ros/point_cloud.h>
//#include <sensor_msgs/msg/point_cloud.h>
#include <std_msgs/msg/string.h>
#include <lslidar_msgs/msg/lslidar_packet.hpp>
#include <lslidar_msgs/msg/lslidar_point.hpp>
#include <lslidar_msgs/msg/lslidar_scan.hpp>
#include <lslidar_msgs/srv/lslidarcontrol.hpp>

/******************************************************************************
 * This file is part of lslidar driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

namespace lslidar_ch_driver {

    static const double DISTANCE_RESOLUTION = 0.0000390625; /**< meters */


    struct PointXYZIRT {
        PCL_ADD_POINT4D
        float intensity;
        uint16_t ring;
        float time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

    class LslidarChDriver : public rclcpp::Node {
    private:

/*        union two_bytes{
            int16_t value;
            char bytes[2];
        };

        struct Point {
            uint8_t vertical_line;        //0-127
            uint8_t azimuth_1;
            uint8_t azimuth_2;      ///< 1500-16500, divide by 100 to get degrees
            uint8_t distance_1;
            uint8_t distance_2;
            uint8_t distance_3;
            uint8_t intensity;
        };*/



        struct Firing {
            //double vertical_angle;
            int vertical_line;
            double azimuth;
            double distance;
            float intensity;
            double time;
        };

    public:
        LslidarChDriver();

        LslidarChDriver(const rclcpp::NodeOptions &options);

        ~LslidarChDriver();

        bool initialize();

        void publishLaserScan();

        void publishPointCloud();

        int convertCoordinate(struct Firing lidardata);

        bool polling();

        void difopPoll(void);

        void initTimeStamp(void);

        bool isPointInRange(const double &distance) {
            return (distance >= min_range && distance <= max_range);
        }

        //void getFPGA_GPSTimeStamp(lslidar_msgs::LslidarChPacketPtr &packet);

        typedef std::shared_ptr<LslidarChDriver> LslidarChDriverPtr;
        typedef std::shared_ptr<const LslidarChDriver> LslidarChDriverConstPtr;

    private:


        bool loadParameters();

        bool createRosIO();

        //socket Parameters
        int msop_udp_port;
        int difop_udp_port;

        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;

        // Converter convtor_
        std::shared_ptr<std::thread> difop_thread_;

        // Ethernet relate variables
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string pointcloud_topic;
        std::string lidar_type;

        in_addr lidar_ip;

        int socket_id;

        bool add_multicast;
        bool pcl_type;
        std::string dump_file;
        double vertical_angle[4];

        double min_range;
        double max_range;
        double angle_disable_min;
        double angle_disable_max;
        int channel_num;

        int echo_num;

        double horizontal_angle_resolution;


        // ROS related variables
        std::mutex pointcloud_lock;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub;

        lslidar_msgs::msg::LslidarScan::UniquePtr sweep_data;
        lslidar_msgs::msg::LslidarScan::UniquePtr sweep_data_bak;


        // add for time synchronization
        bool use_time_service;
        bool use_gps;
       // std::string time_service_mode;
        bool publish_laserscan;
       // bool gain_prism_angle;
        int lidarpoint_count;
        uint64_t packet_timestamp_s;
        uint64_t packet_timestamp_ns;
        double packet_timestamp;
        double last_packet_timestamp;
        double point_cloud_timestamp;
        double point_time;
        double packet_rate;


        unsigned char packetTimeStamp[10];
        struct tm cur_time;

        //   ros::Time timeStamp;

        //   ros::Time packet_timeStamp;

        double packet_interval_time;
        //   Firing firings[171];
    };

    typedef LslidarChDriver::LslidarChDriverPtr LslidarChDriverPtr;
    typedef LslidarChDriver::LslidarChDriverConstPtr LslidarChDriverConstPtr;
    typedef PointXYZIRT VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;

} // namespace lslidar_driver

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_ch_driver::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)
                                          (std::uint16_t, ring, ring)
                                          (float , time, time)
)

#endif // _LSLIDAR_Ch_DRIVER_H_

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
    static const double sqrt_0_5 = sqrt(0.5);
    static const double pow1 = -3.6636 * pow(10, -7);
    static const double pow2 = 5.2766 * pow(10, -5);
    static const double pow3 = 1.4507 * pow(10, -4);
    static const double big_angle[32] = {-17, -16, -15, -14, -13, -12, -11, -10,
                                         -9, -8, -7, -6, -5, -4.125, -4, -3.125,
                                         -3, -2.125, -2, -1.125, -1, -0.125, 0, 0.875,
                                         1, 1.875, 2, 3, 4, 5, 6, 7
    };
    static const double big_angle_ch128s1[32] = {
            -12, -11, -10, -9, -8, -7, -6, -5,
            -4.125, -4, -3.125, -3, -2.125, -2, -1.125, -1,
            -0.125, 0, 0.875, 1, 1.875, 2, 3, 4,
            5, 6, 7, 8, 9, 10, 11, 12
    };

    static const double big_angle_cx126s3[42] = {
            -12.0,-11.4,-10.8,-10.2,-9.6,-9.0,-8.4,-7.8,-7.2,-6.6,-6.0,-5.4,-4.8,-4.2,
            -3.6,-3.0,-2.4,-1.8,-1.2,-0.6,0.0,0.6,1.2,1.8,2.4,3.0,3.6,4.2,4.8,5.4,6.0,
            6.6,7.2,7.8,8.4,9.0,9.6,10.2,10.8,11.4,12.0,12.6
    };


    //static const double big_angle_ch16x1[16] = {-1.0,-0.75,-0.50,-0.25,0.0,0.25,0.50,0.75,1.0,1.25,1.50,1.75,2.0,2.25,2.50,2.75};
    static const double big_angle_ch16x1[4] = {-1.0, 0.0, 1.0, 2.0};


/*    static const double scan_mirror_altitude[4] = {
            -0.0,
            0.005759586531581287,
            0.011693705988362009,
            0.017453292519943295,
    };

    static const double sin_scan_mirror_altitude[4] = {
            std::sin(scan_mirror_altitude[0]), std::sin(scan_mirror_altitude[1]),
            std::sin(scan_mirror_altitude[2]), std::sin(scan_mirror_altitude[3]),
    };*/
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
            int azimuth;
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

        int convertCoordinate(struct Firing &lidardata);

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
        double prism_angle[4];
        double prism_offset;
        double min_range;
        double max_range;
        int angle_disable_min;
        int angle_disable_max;
        int channel_num;
        int channel_num1;
        int channel_num2;
        int echo_num;

        double horizontal_angle_resolution;
        double sin_list[36000]{};
        double cos_list[36000]{};
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_bak_;
        sensor_msgs::msg::LaserScan::UniquePtr  scan_msg;
        sensor_msgs::msg::LaserScan::UniquePtr scan_msg_bak;
        uint point_size;


        // ROS related variables
        std::mutex pointcloud_lock;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub;

        //lslidar_msgs::msg::LslidarScan::UniquePtr sweep_data;
        //lslidar_msgs::msg::LslidarScan::UniquePtr sweep_data_bak;


        // add for time synchronization
        bool use_time_service;
        std::string time_service_mode;
        bool publish_laserscan;
        bool gain_prism_angle;
        bool is_update_difop_packet;
        uint32_t packet_timestamp_s;
        uint32_t packet_timestamp_ns;
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

        double sin_theta_1[128];
        double sin_theta_2[128];
        double cos_theta_1[128];
        double cos_theta_2[128];

        double ch64w_sin_theta_1[128];
        double ch64w_sin_theta_2[128];
        double ch64w_cos_theta_1[128];
        double ch64w_cos_theta_2[128];

        double cx126s3_sin_theta_1[126];
        double cx126s3_sin_theta_2[126];
        double cx126s3_cos_theta_1[126];
        double cx126s3_cos_theta_2[126];

        double ch16x1_sin_theta_1[16];
        double ch16x1_sin_theta_2[16];
        double ch16x1_cos_theta_1[16];
        double ch16x1_cos_theta_2[16];
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
                                          (float, time, time)
)

#endif // _LSLIDAR_Ch_DRIVER_H_

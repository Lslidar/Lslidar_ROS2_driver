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

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "input.h"
#include "lslidar_log.h"
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

    constexpr double single_interval_time = 1.0 / 1197.0;
    constexpr double double_interval_time = 1.0 / 1199.0;
    constexpr double DEG_TO_RAD  = 0.017453292;
    constexpr double RAD_TO_DEG  = 57.29577951;
    constexpr double SINGLE_ECHO = 0.005847953;
    constexpr double DOUBLE_ECHO = 0.009174312;
    constexpr double DISTANCE_RESOLUTION = 0.0000390625; /**< meters */

    constexpr double sqrt_0_5 = sqrt(0.5);
    constexpr double pow1 = -3.6636 * pow(10, -7);
    constexpr double pow2 = 5.2766 * pow(10, -5);
    constexpr double pow3 = 1.4507 * pow(10, -4);

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
        struct Firing {
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

        typedef std::shared_ptr<LslidarChDriver> LslidarChDriverPtr;
        typedef std::shared_ptr<const LslidarChDriver> LslidarChDriverConstPtr;

    private:

        bool loadParameters();

        void outputParameters();

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
        int scan_start_angle;
        int scan_end_angle;
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

        double packet_interval_time;
        double point_interval_time;

        double ch1w_sin_theta_1[8];
        double ch1w_sin_theta_2[8];
        double ch1w_cos_theta_1[8];
        double ch1w_cos_theta_2[8];
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

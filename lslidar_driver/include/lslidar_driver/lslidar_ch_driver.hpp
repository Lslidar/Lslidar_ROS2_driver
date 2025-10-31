/******************************************************************************
 * This file is part of lslidar_driver.
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

#ifndef LSLIDAR_CH_DRIVER_HPP
#define LSLIDAR_CH_DRIVER_HPP

#include "lslidar_driver/lslidar_driver.hpp"

#include <cmath>
#include <unordered_map>

namespace lslidar_driver {
    constexpr double CH_SINGLE_ECHO = 0.005847953;
    constexpr double CH_DOUBLE_ECHO = 0.009174312;

    constexpr double CH_DISTANCE_RESOLUTION = 0.0000390625;
    constexpr double sqrt_0_5 = std::sqrt(0.5);
    constexpr double pow1 = -0.00000036636;
    constexpr double pow2 = 0.000052766;
    constexpr double pow3 = 0.00014507;
    
    constexpr double big_angle[32] = {
        -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4.125, -4, -3.125,
        -3, -2.125, -2, -1.125, -1, -0.125, 0, 0.875, 1, 1.875, 2, 3, 4, 5, 6, 7
    };

    constexpr double big_angle_cx6s3[2] = {-0.6, 0.0};

    constexpr double big_angle_ch16x1[4] = {-1.0, 0.0, 1.0, 2.0};

    constexpr double big_angle_ch128s1[32] = {
        -12, -11, -10, -9, -8, -7, -6, -5, -4.125, -4, -3.125, -3, -2.125, -2, -1.125, -1,
        -0.125, 0, 0.875, 1, 1.875, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12
    };

    constexpr double big_angle_cx126s3[42] = {
        -12.0, -11.4, -10.8, -10.2, -9.6, -9.0, -8.4, -7.8, -7.2, -6.6, -6.0, -5.4, -4.8, -4.2,
        -3.6, -3.0, -2.4, -1.8, -1.2, -0.6, 0.0, 0.6, 1.2, 1.8, 2.4, 3.0, 3.6, 4.2, 4.8, 5.4, 6.0,
        6.6, 7.2, 7.8, 8.4, 9.0, 9.6, 10.2, 10.8, 11.4, 12.0, 12.6
    };

    // CH32A
    constexpr double scan_laser_altitude_1[8] = {-6.67, -5.33, -4, -2.67, -1.33, 0, 1.33, 2.67};

    constexpr double scan_mirror_altitude[4] = {
        -0.0,
        0.005759586531581287,
        0.011693705988362009,
        0.017453292519943295,
    };

    constexpr double sin_scan_mirror_altitude[4] = {
        std::sin(scan_mirror_altitude[0]), std::sin(scan_mirror_altitude[1]),
        std::sin(scan_mirror_altitude[2]), std::sin(scan_mirror_altitude[3]),
    };

    struct Firing {
        uint16_t vertical_line;
        int azimuth;
        double distance;
        float intensity;
        double time;
    };

    class LslidarChDriver : public LslidarDriver {
    private:
        std::unordered_map<std::string, std::function<void(Firing&)>> coordinateConverters;

        std::function<void(Firing&)> currentConverter;  // 当前雷达的处理函数

        std::function<void(const lslidar_msgs::msg::LslidarPacket::UniquePtr&)> packetProcess;

    public:
        LslidarChDriver(rclcpp::Node::SharedPtr node);

        virtual ~LslidarChDriver() = default;

        bool loadParameters() override;

        void outputParameters() override;

        bool createRosIO() override;

        void initTimeStamp() override;

        bool initialize() override;

        bool poll() override;

        void difopPoll();

        bool getLidarEcho();

        void publishLaserScan();

        void publishPointcloud();

        void initAngleConfig();

        void convertCoordinate(Firing &data) { currentConverter(data); }

        void convertCoordinate_cx1s3(struct Firing &lidardata);

        void convertCoordinate_cx6s3(struct Firing &lidardata);

        void convertCoordinate_ch16x1(struct Firing &lidardata);

        void convertCoordinate_ch32a(struct Firing &lidardata);

        void convertCoordinate_ch64w(struct Firing &lidardata);

        void convertCoordinate_cb64s1_a(struct Firing &lidardata);

        void convertCoordinate_cx126s3(struct Firing &lidardata);

        void convertCoordinate_ch128(struct Firing &lidardata); //ch128x1 ch128s1

        void convertCoordinate_cx128s2(struct Firing &lidardata); //cx128s2

        void convertCoordinate_ch256(struct Firing &lidardata);

        void packetProcessSingle(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet);

        void packetProcessDouble(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet);

        void lidarEchoMode(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet);

        void updateLaserscan(float x, float y, float intensity);

        inline void resetVariables_ch64w() {
            x = 0.0; y = 0.0; z = 0.0; sin_theat = 0.0; cos_theat = 0.0; _R_ = 0.0; cos_xita = 0.0; sin_xita = 0.0;
            cos_H_xita = 0.0; sin_H_xita = 0.0; cos_xita_F = 0.0; sin_xita_F = 0.0; add_distance = 0.0;
        }

        inline void resetVariables_cb64s1_a() {
            x = 0.0; y = 0.0; z = 0.0; sin_theat = 0.0; cos_theat = 0.0; _R_ = 0.0; cos_xita = 0.0; sin_xita = 0.0;
        }

        typedef std::shared_ptr<LslidarChDriver> LslidarChDriverPtr;
        typedef std::shared_ptr<const LslidarChDriver> LslidarChDriverConstPtr;

    private:
        int echo_byte = 1211;

        double prism_angle[4];
        double sin_list[36000]{};
        double cos_list[36000]{};

        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        sensor_msgs::msg::LaserScan::UniquePtr scan_msg;
        sensor_msgs::msg::LaserScan::UniquePtr scan_msg_bak;

        uint point_size;
        double prism_offset;
        int scan_start_angle;
        int scan_end_angle;
        int channel_num;
        int channel_num1;
        int channel_num2;
        int echo_mode;

        double horizontal_angle_resolution;
        
        std::string time_service_mode;

        bool use_time_service;
        bool publish_laserscan;
        bool gain_prism_angle;
        bool add_multicast;
        bool packetType;

        uint32_t packet_timestamp_s;
        uint32_t packet_timestamp_ns;
        double packet_timestamp;
        double last_packet_timestamp;
        double point_cloud_timestamp;
        double packet_interval_time;
        double point_interval_time;
        double point_time;
        double packet_rate;

        std::mutex pointcloud_lock;

        unsigned char packetTimeStamp[10];
        struct tm cur_time;

        double sin_theta_1[128];
        double sin_theta_2[128];
        double cos_theta_1[128];
        double cos_theta_2[128];
        double ch256_sin_theta_1[256];
        double ch256_sin_theta_2[256];
        double ch256_cos_theta_1[256];
        double ch256_cos_theta_2[256];

        double ch64w_sin_theta_1[128];
        double ch64w_sin_theta_2[128];
        double ch64w_cos_theta_1[128];
        double ch64w_cos_theta_2[128];

        double cb64s1_A_sin_theta_1[128];
        double cb64s1_A_sin_theta_2[128];
        double cb64s1_A_cos_theta_1[128];
        double cb64s1_A_cos_theta_2[128];

        double ch16x1_sin_theta_1[16];
        double ch16x1_sin_theta_2[16];
        double ch16x1_cos_theta_1[16];
        double ch16x1_cos_theta_2[16];

        double cx126s3_sin_theta_1[126];
        double cx126s3_sin_theta_2[126];
        double cx126s3_cos_theta_1[126];
        double cx126s3_cos_theta_2[126];

        double cx6s3_sin_theta_1[6];
        double cx6s3_sin_theta_2[6];
        double cx6s3_cos_theta_1[6];
        double cx6s3_cos_theta_2[6];

        double sin_scan_laser_altitude[8]; // CH32A

        double x, y, z;
        double sin_theat;
        double cos_theat;
        double _R_;

        double cos_xita;
        double sin_xita;
        double cos_H_xita;
        double sin_H_xita;
        double cos_xita_F;
        double sin_xita_F;
        double add_distance;

        double x_offset;
        double y_offset;
        double z_offset;
        double roll;
        double pitch;
        double yaw;
    };

    typedef LslidarChDriver::LslidarChDriverPtr LslidarChDriverPtr;
    typedef LslidarChDriver::LslidarChDriverConstPtr LslidarChDriverConstPtr;
    typedef PointXYZIRT VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;

} // namespace lslidar_driver

#endif // LSLIDAR_CH_DRIVER_HPP

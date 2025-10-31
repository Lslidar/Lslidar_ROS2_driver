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

#ifndef LSLIDAR_X10_DRIVER_HPP
#define LSLIDAR_X10_DRIVER_HPP

#include <vector>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"

#include "lslidar_driver/lsiosr.hpp"
#include "lslidar_driver/lslidar_driver.hpp"
#include "lslidar_driver/data_acquisition_strategy.hpp"

namespace lslidar_driver {
    static const float INITIAL_RANGE = std::numeric_limits<float>::infinity();
    static const float INITIAL_INTENSITIE = std::numeric_limits<float>::quiet_NaN();

    static const float ANGLE_INTERVAL_M10 = 15.0; 
    static const float DISTANCE_RESOLUTION_X10 = 0.001f;
    static const int POINT_LEN_M10 = 2;
    static const int POINT_LEN_N10 = 3;

    static const int BLOCKS_PER_PACKET_N301 = 12;            // 每个数据包中数据块个数
    static const int SIZE_BLOCK_N301 = 100;                  // 每个数据块长度   2字节标准位 2字节方位角 6字节时间 90字节点信息
    static const int RAW_SCAN_SIZE_N301 = 3;                 // 每个点长度 距离2字节  强度1字节
   
    static const int SCANS_PER_BLOCK_N301_1_7 = 31;
    static const int FIRING_TOFFSET_N301_1_7 = 30;              // 1.7协议 每个数据块中点数
    static const float DISTANCE_RESOLUTION_N301_1_7 = 0.004f;   // 1.7协议 距离分辨率

    static const int FIRING_TOFFSET_N301_1_6 = 2;
    static const float DISTANCE_RESOLUTION_N301_1_6 = 0.002f;   // 1.6协议 距离分辨率

    static const float M10_PULSE_FREQUENCY = 10000.0;   // M10 M10GPS  10K
    static const float M10P_PULSE_FREQUENCY = 20000.0;  // M10P  20K
    static const float N10_PULSE_FREQUENCY = 4500.0;    // N10   4.5K
    static const float N10PLUS_PULSE_FREQUENCY = 5400.0;   // N10PLUS  5.4K
    static const float N301_PULSE_FREQUENCY = 20000.0;  // N301 20K

    static const int M10_MOTOR_FREQUENCY = 10;
    static const int M10P_MOTOR_FREQUENCY = 12;
    static const int N10_MOTOR_FREQUENCY = 10;
    static const int N301_MOTOR_FREQUENCY = 10;

    typedef struct {
        int azimuth;
        float distance;
        float intensity;
    } FiringX10;

    class LslidarX10Driver : public LslidarDriver {
    public:
        LslidarX10Driver(rclcpp::Node::SharedPtr node); 

        virtual ~LslidarX10Driver() = default;

        bool loadParameters() override;

        void outputParameters() override;

        bool createRosIO() override;

        void initTimeStamp() override;

        bool initialize() override;

        bool poll() override;

        bool initAngleConfig();

        void publishLiadrData();

        void publishMultiEchoLaserScan();

        std::function<bool(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet, int packet_size)> checkPacketValidity;

        bool checkPacketValidityM10(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet, int packet_size) const;

        bool checkPacketValidityN10(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet, int packet_size) const;

        bool checkPacketValidityN301(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet, int packet_size) const;

        std::function<void(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet)> decodePacket;

        void decodePacketM10(lslidar_msgs::msg::LslidarPacket::UniquePtr &packetlidardata);

        void decodePacketN10(lslidar_msgs::msg::LslidarPacket::UniquePtr &packetlidardata);

        void decodePacketN10Plus(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet);

        void decodePacketN301_1_6(lslidar_msgs::msg::LslidarPacket::UniquePtr &packetlidardata);

        void decodePacketN301_1_7(lslidar_msgs::msg::LslidarPacket::UniquePtr &packetlidardata);

        bool isPointValid(const int fir_idx) const;

        bool isNoisePoint(size_t curr_idx, size_t end_fir_idx) const;

        void resetMultiEchoLaserScan();

        void pointcloudToLaserscan(const sensor_msgs::msg::PointCloud2 &cloud_msg, sensor_msgs::msg::LaserScan &output_scan);

        bool judgmentProtocol(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet);

        bool determineN301Model();

        bool configureParameters();

        void motorControl(const std_msgs::msg::Int8::SharedPtr msg);

    public:
        std::shared_ptr<DataAcquisitionStrategy> data_acquisition_strategy_;
        std::shared_ptr<LSIOSR> serial_input_;
        std::string serial_port_;
        std::string san_topic_;

        BaudRate baud_rate;

        rclcpp::Publisher<sensor_msgs::msg::MultiEchoLaserScan>::SharedPtr multiecho_scan_pub_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr motor_control_sub_;

        std::vector<FiringX10> points;
        std::mutex pointcloud_lock;

        uint16_t last_azimuth;
        uint16_t packet_points_max;
        uint16_t actual_points;
        uint16_t motor_speed;
        uint16_t points_size;

        int angle_disable_min{};
        int angle_disable_max{};
        int angle_able_min{};
        int angle_able_max{};
        float cos_azimuth_table[36000]{};
        float sin_azimuth_table[36000]{};

        float n301_protocol;
        int N10Plus_hz;
        int packet_length;
        int data_bits_start;
        int angle_bits_start;
        int end_angle_bits_start;

        std::atomic<bool> motor_running{true};
        bool is_first_sweep;
        bool publish_scan;
        bool publish_multiecholaserscan;
        bool enable_noise_filter;
        bool is_valid_point;
        bool use_high_precision;

        struct tm cur_time;
        int YEAR;
        int MONTH;
        int DAY;
        uint64_t packet_time_s;
        uint64_t packet_time_ns;
        unsigned char packetTimeStamp[10]{};
        rclcpp::Time timeStamp;
        double sweep_end_time;
        double current_packet_time;
        double last_packet_time;

        sensor_msgs::msg::MultiEchoLaserScan::UniquePtr multiecho_scan_;
        sensor_msgs::msg::MultiEchoLaserScan::UniquePtr multiecho_scan_bak_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_bak_;
    };
} // namespace lslidar_driver


#endif // _LSLIDAR_X10_DRIVER_HPP

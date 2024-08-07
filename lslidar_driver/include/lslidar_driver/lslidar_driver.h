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

#include "input.h"
#include "lslidar_log.h"
#include <string>
#include <thread>
#include <memory>
#include <regex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.h>

#include <lslidar_msgs/msg/lslidar_packet.hpp>
#include <lslidar_msgs/msg/lslidar_point.hpp>
#include <lslidar_msgs/msg/lslidar_scan.hpp>
#include <lslidar_msgs/srv/data_ip.hpp>
#include <lslidar_msgs/srv/data_port.hpp>
#include <lslidar_msgs/srv/destination_ip.hpp>
#include <lslidar_msgs/srv/dev_port.hpp>
#include <lslidar_msgs/srv/lslidar_control.hpp>
#include <lslidar_msgs/srv/motor_control.hpp>
#include <lslidar_msgs/srv/motor_speed.hpp>
#include <lslidar_msgs/srv/time_service.hpp>


namespace lslidar_driver {
//raw lslidar packet constants and structures
    static const int RAW_SCAN_SIZE = 3;
    static const int SCANS_PER_BLOCK = 32;
    static const int BLOCK_DATA_SIZE = SCANS_PER_BLOCK * RAW_SCAN_SIZE;
    static const double DISTANCE_RESOLUTION = 0.01; //meters

// special defines for lslidarlidar support
    static const int FIRINGS_PER_BLOCK = 2;
    static const int SCANS_PER_FIRING = 16;
    static const int SCANS_PER_FIRING_CX = 32;
    static const int FIRING_TOFFSET = 32;
    static const int BLOCKS_PER_PACKET = 12;
    static const int FIRINGS_PER_PACKET_CX = BLOCKS_PER_PACKET;
    static const int SCANS_PER_PACKET = SCANS_PER_FIRING * FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET; //384
    //unit:meter
    static const float R1_ = 0.0361f;        // C16 4.0 or C32 4.0
    static const float R1_C32W = 0.03416f;   // 新C32W   byte[1211] = 0x47
    static const float R1_90 = 0.0209f;      // 90度
    static const double R2_ = 0.0431;        // c16 3.0
    static const double R3_ = 0.0494;        // c32 3.0 

    static const int conversionAngle_ = 2025;
    static const int conversionAngle_90 = 2776; // 90度
    static const int conversionAngle_C16_3 = 1468;  //C16 3.0
    static const int conversionAngle_C32_3 = 1298;  //C32 3.0

    //c32 32度
    static const float c32_vertical_angle[32] = {
            -16.0f, -8.0f, 0.0f, 8.0f, -15.0f, -7.0f, 1.0f, 9.0f,
            -14.0f, -6.0f, 2.0f, 10.0f, -13.0f, -5.0f, 3.0f, 11.0f,
            -12.0f, -4.0f, 4.0f, 12.0f, -11.0f, -3.0f, 5.0f, 13.0f,
            -10.0f, -2.0f, 6.0f, 14.0f, -9.0f, -1.0f, 7.0f, 15.0f};

    //c32 70度  0x45    W_P
    static const float c32wp_vertical_angle[32] = {
            -51.0f, -31.0f, -9.0f, 3.0f, -48.5f, -28.0f, -7.5f, 4.5f,
            -46.0f, -25.0f, -6.0f, 6.0f, -43.5f, -22.0f, -4.5f, 7.5f,
            -41.0f, -18.5f, -3.0f, 9.0f, -38.5f, -15.0f, -1.5f, 11.0f,
            -36.0f, -12.0f, 0.0f, 13.0f, -33.5f, -10.5f, 1.5f, 15.0f};
    
    //c32 70度  0x46    w 
    static const float c32_70_vertical_angle[32] = {
            -54.0f, -31.0f, -8.0f, 2.66f, -51.5f, -28.0f, -6.66f, 4.0f,
            -49.0f, -25.0f, -5.33f, 5.5f, -46.0f, -22.0f, -4.0f, 7.0f,
            -43.0f, -18.5f, -2.66f, 9.0f, -40.0f, -15.0f, -1.33f, 11.0f,
            -37.0f, -12.0f, 0.0f, 13.0f, -34.0f, -10.0f, 1.33f, 15.0f};
    
    //c32 70度  0x47    wn wb
    static const float c32wn_vertical_angle2[32] = {
            -54.7f, -31.0f, -9.0f, 3.0f, -51.5f, -28.0f, -7.5f, 4.5f,
            -49.0f, -25.0f, -6.0f, 6.0f, -46.0f, -22.0f, -4.5f, 7.5f,
            -43.0f, -18.5f, -3.0f, 9.0f, -40.0f, -15.0f, -1.5f, 11.0f,
            -37.0f, -12.0f, 0.0f, 13.0f, -34.0f, -10.5f, 1.5f, 15.0f};

    //c32 70度  0x47    wn wb
    static const float c32wb_vertical_angle2[32] = {
            -54.7f, -9.0f, -31.0f, 3.0f, -51.5f, -7.5f, -28.0f, 4.5f,
            -49.0f, -6.0f, -25.0f, 6.0f, -46.0f, -4.5f, -22.0f, 7.5f,
            -43.0f, -3.0f, -18.5f, 9.0f, -40.0f, -1.5f, -15.0f, 11.0f,
            -37.0f, 0.0f, -12.0f, 13.0f, -34.0f, 1.5f, -10.5f, 15.0f};

    //c32 90度
    static const float c32_90_vertical_angle[32] = {
            2.487f, 25.174f, 47.201f, 68.819f, 5.596f, 27.811f, 49.999f, 71.525f,
            8.591f, 30.429f, 52.798f, 74.274f, 11.494f, 33.191f, 55.596f, 77.074f,
            14.324f, 36.008f, 58.26f, 79.938f, 17.096f, 38.808f, 60.87f, 82.884f,
            19.824f, 41.603f, 63.498f, 85.933f, 22.513f, 44.404f, 66.144f, 89.105f};

    static const float c32rn_vertical_angle[32] = {
            2.487f, 47.201f, 25.174f, 68.819f, 5.596f, 49.999f, 27.811f, 71.525f,
            8.591f, 52.798f, 30.429f, 74.274f, 11.494f, 55.596f, 33.191f, 77.074f,
            14.324f, 58.26f, 36.008f, 79.938f, 17.096f, 60.87f, 38.808f, 82.884f,
            19.824f, 63.498f, 41.603f, 85.933f, 22.513f, 66.144f, 44.404f, 89.105f};

    static float c16_vertical_angle[16] = {-16.0f, 0.0f, -14.0f, 2.0f, -12.0f, 4.0f, -10.0f, 6.0f,
                                           -8.0f, 8.0f, -6.0f, 10.0f, -4.0f, 12.0f, -2.0f, 14.0f};

    static const int c16_remap_angle[16] = {0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15};

    static const float c8_vertical_angle[8] = {-10.0f, 4.0f, -8.0f, 8.0f, -4.0f, 10.0f, 0.0f, 12.0f};

    static const float c8f_vertical_angle[8] = {-2.0f, 6.0f, 0.0f, 8.0f, 2.0f, 10.0f, 4.0f, 12.0f};
    
    static const float ckm8_vertical_angle[8] = {-12.0f, 4.0f, -8.0f, 8.0f, -4.0f, 10.0f, 0.0f, 12.0f};

    static const float c1_vertical_angle[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    //c16 3.0
    static const float c16_30_vertical_angle[16] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};

    // c32 3.0
    static const float c32_30_vertical_angle[32] = {-16, 0, -15, 1, -14, 2, -13, 3, -12, 4, -11, 5, -10, 6, -9, 7, -8,
                                                   8, -7, 9, -6, 10, -5, 11, -4, 12, -3, 13, -2, 14, -1, 15};

    static const uint8_t adjust_angle_index[32] = {0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8, 24, 9,
                                                   25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31};

    union TwoBytes {
        uint16_t distance;
        uint8_t bytes[2];
    };

    struct RawBlock {
        uint16_t header;
        uint16_t rotation;  //0-35999
        uint8_t data[BLOCK_DATA_SIZE];
    };

    struct RawPacket {
        RawBlock blocks[BLOCKS_PER_PACKET];
        uint8_t time_stamp[10];
        uint8_t factory[2];
    };

    struct FiringCX {
        uint16_t firing_azimuth[FIRINGS_PER_PACKET_CX];
        int azimuth[SCANS_PER_PACKET];
        double distance[SCANS_PER_PACKET];
        double intensity[SCANS_PER_PACKET];
    };

    struct PointXYZIRT {
        PCL_ADD_POINT4D;
        PCL_ADD_INTENSITY
        std::uint16_t ring;
        float time;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //make sure our new allocators are aligned
    } EIGEN_ALIGN16; //enforce SSE padding for correct memory alignment

    static std::string lidar_type;

    class LslidarDriver : public rclcpp::Node {
    public:
        LslidarDriver();

        LslidarDriver(const rclcpp::NodeOptions &options);

        virtual ~LslidarDriver() {}

        bool checkPacketValidity(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet);

        //check if a point is in the required range
        //bool isPointInRange(const double &distance);

        bool loadParameters();

        void outputParameters();

        void initTimeStamp();

        bool createRosIO();

        void publishPointcloud();

        void publishScan();

        bool powerOn(std::shared_ptr<lslidar_msgs::srv::LslidarControl::Request> req,
                     std::shared_ptr<lslidar_msgs::srv::LslidarControl::Response> res);

        bool motorControl(std::shared_ptr<lslidar_msgs::srv::MotorControl::Request> req,
                          std::shared_ptr<lslidar_msgs::srv::MotorControl::Response> res);

        bool motorSpeed(std::shared_ptr<lslidar_msgs::srv::MotorSpeed::Request> req,
                        std::shared_ptr<lslidar_msgs::srv::MotorSpeed::Response> res);

        bool timeService(std::shared_ptr<lslidar_msgs::srv::TimeService::Request> req,
                         std::shared_ptr<lslidar_msgs::srv::TimeService::Response> res);

        bool setDataPort(std::shared_ptr<lslidar_msgs::srv::DataPort::Request> req,
                         std::shared_ptr<lslidar_msgs::srv::DataPort::Response> res);

        bool setDevPort(std::shared_ptr<lslidar_msgs::srv::DevPort::Request> req,
                        std::shared_ptr<lslidar_msgs::srv::DevPort::Response> res);

        bool setDataIp(std::shared_ptr<lslidar_msgs::srv::DataIp::Request> req,
                       std::shared_ptr<lslidar_msgs::srv::DataIp::Response> res);

        bool setDestinationIp(std::shared_ptr<lslidar_msgs::srv::DestinationIp::Request> req,
                              std::shared_ptr<lslidar_msgs::srv::DestinationIp::Response> res);

        static void setPacketHeader(unsigned char *config_data);

        bool sendPacketTolidar(unsigned char *config_data) const;

        void difopPoll();

        void pollThread(void);

        bool poll();

        void pointcloudToLaserscan(const sensor_msgs::msg::PointCloud2 &cloud_msg, sensor_msgs::msg::LaserScan &output_scan);

        bool initialize();

        void decodePacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet);

        bool determineLidarType();

        //int calculateRemappedScanIndex(int fir_idx, int ring_);

    public:
        int msop_udp_port{};
        int difop_udp_port{};
        int scan_num{};
        int angle_disable_min{};
        int angle_disable_max{};
        int angle_able_min{};
        int angle_able_max{};
        uint16_t last_azimuth;
        uint64_t packet_time_s;
        uint64_t packet_time_ns;

        in_addr lidar_ip;
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string dump_file;
        std::string pointcloud_topic;

        bool use_time_service;
        bool pcl_type;
        bool publish_scan;
        bool coordinate_opt;
        bool is_first_sweep;
        bool add_multicast;

        double distance_unit;
        double min_range;
        double max_range;
        double sweep_end_time;
        double angle_base;
        double cos_azimuth_table[36000]{};
        double sin_azimuth_table[36000]{};

        std::shared_ptr <Input> msop_input_;
        std::shared_ptr <Input> difop_input_;
        std::shared_ptr <std::thread> difop_thread_;
        std::shared_ptr <std::thread> poll_thread_;
        
        std::mutex pointcloud_lock;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
        rclcpp::Service<lslidar_msgs::srv::LslidarControl>::SharedPtr lslidar_control_service_;      //上下电
        rclcpp::Service<lslidar_msgs::srv::MotorControl>::SharedPtr motor_control_service_;          //雷达转动/停转
        rclcpp::Service<lslidar_msgs::srv::TimeService>::SharedPtr time_service_;                    //授时
        rclcpp::Service<lslidar_msgs::srv::MotorSpeed>::SharedPtr motor_speed_service_;              //雷达频率
        rclcpp::Service<lslidar_msgs::srv::DataPort>::SharedPtr data_port_service_;                  //数据包端口
        rclcpp::Service<lslidar_msgs::srv::DevPort>::SharedPtr dev_port_service_;                    //设备包端口
        rclcpp::Service<lslidar_msgs::srv::DataIp>::SharedPtr data_ip_service_;                      //数据包ip
        rclcpp::Service<lslidar_msgs::srv::DestinationIp>::SharedPtr destination_ip_service_;        //设备包ip

        unsigned char difop_data[1206];
        unsigned char packetTimeStamp[10];
        struct tm cur_time;
        rclcpp::Time timeStamp;
        rclcpp::Time timeStamp_bak;
        int return_mode;
        int lidar_number_;
        int point_num;
        double packet_rate;
        double current_packet_time;
        double last_packet_time;
        
        FiringCX firings;
        double scan_altitude[32] = {0};
        double cos_scan_altitude[32] = {0};
        double sin_scan_altitude[32] = {0};
        double horizontal_angle_resolution;
        double config_vertical_angle_32[32] = {0};
        double config_vertical_angle_tmp[32] = {0};
        double config_vertical_angle_16[16] = {0};
        float msc16_adjust_angle[16] = {0.0f};
        float msc16_offset_angle[16] = {0.0f};
        float R1 = 0.0f;
        int adjust_angle[4];
        int config_vert_num;
        
        std::atomic<bool> is_get_difop_;
        std::atomic<bool> start_process_msop_;
        std::atomic<int> time_service_mode_ {0};
        
        bool is_new_c32w_ = true;
        bool is_msc16 = true;
        bool angle_change{};
        int ring_;
        int fpga_type{};
        int conversionAngle{};
        int config_vertical_angle_flag{};
        int remapped_scan_idx{};
        int packet_num{};
        int number_threshold = 10;
        uint point_size;

        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_bak_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        sensor_msgs::msg::LaserScan::UniquePtr scan_msg;
        sensor_msgs::msg::LaserScan::UniquePtr scan_msg_bak;
    };

    typedef PointXYZIRT VPoint;
    typedef pcl::PointCloud <VPoint> VPointcloud;

}  // namespace lslidar_driver

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_driver::PointXYZIRT,
(float, x, x)
(float, y, y)
(float, z, z)
(float, intensity, intensity)
(std::uint16_t, ring, ring)
(float, time, time))

#endif

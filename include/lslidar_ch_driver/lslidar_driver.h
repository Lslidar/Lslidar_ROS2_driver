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
#include <map>
#include "input.h"
#include <thread>
#include <memory>
#include <functional>
#include <unordered_map>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.h>
#include <lslidar_ch_driver/msg/lslidar_packet.hpp>
#include "ThreadPool.h"

namespace lslidar_ch_driver {
    constexpr double single_interval_time = 1.0 / 1197.0;
    constexpr double double_interval_time = 1.0 / 1199.0;
    constexpr double DEG_TO_RAD  = 0.017453292;
    constexpr double RAD_TO_DEG  = 57.29577951;
    constexpr double SINGLE_ECHO = 0.005847953;
    constexpr double DOUBLE_ECHO = 0.009174312;
    
    constexpr double sqrt_0_5 = sqrt(0.5);
    constexpr double pow1 = -3.6636 * pow(10, -7);
    constexpr double pow2 = 5.2766  * pow(10, -5);
    constexpr double pow3 = 1.4507  * pow(10, -4);
    constexpr double DISTANCE_RESOLUTION = 0.0000390625; /**< meters */

    constexpr double big_angle_cx6s3[2] = {-0.6,0.0};

    constexpr double big_angle_ch16x1[4] = {-1.0, 0.0, 1.0, 2.0};

    constexpr double big_angle[32] = {-17, -16, -15, -14, -13, -12, -11, -10,
                                      -9, -8, -7, -6, -5, -4.125, -4, -3.125,
                                      -3, -2.125, -2, -1.125, -1, -0.125, 0, 0.875,
                                      1, 1.875, 2, 3, 4, 5, 6, 7};

    
    constexpr double big_angle_ch128s1[32] = {-12, -11, -10, -9, -8, -7, -6, -5,
                                              -4.125, -4, -3.125, -3, -2.125, -2, -1.125, -1,
                                              -0.125, 0, 0.875, 1, 1.875, 2, 3, 4,
                                               5, 6, 7, 8, 9, 10, 11, 12};

    
    constexpr double big_angle_cx126s3[42] = {-12.0, -11.4, -10.8, -10.2, -9.6, -9.0, -8.4,
                                               -7.8, -7.2, -6.6, -6.0, -5.4, -4.8, -4.2,
                                               -3.6, -3.0, -2.4, -1.8, -1.2, -0.6, 0.0,
                                                0.6, 1.2, 1.8, 2.4, 3.0, 3.6, 4.2,
                                                4.8, 5.4, 6.0, 6.6, 7.2, 7.8, 8.4,
                                                9.0, 9.6, 10.2, 10.8, 11.4, 12.0, 12.6};

    struct PointXYZIRT {
        PCL_ADD_POINT4D
        float intensity;
        uint8_t ring;
        float time;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned

        PointXYZIRT(float x, float y, float z, float intensity, uint8_t ring, float time)
        : x(x), y(y), z(z), intensity(intensity), ring(ring), time(time) {}

    } EIGEN_ALIGN16;

    class LslidarChDriver : public rclcpp::Node {
    private:
        struct Firing {
            uint8_t vertical_line;
            int azimuth;
            double distance;
            float intensity;
            float time;
        };

        std::map<std::string, int> maxChannelNumbers;

        std::unordered_map<std::string, std::function<void(Firing&)>> coordinateConverters;

        std::function<void(Firing&)> currentConverter;

        std::function<void(const lslidar_ch_driver::msg::LslidarPacket::UniquePtr&)> packetProcess;
        
        std::unique_ptr <ThreadPool> threadPool_;
    
    public:
        LslidarChDriver();

        LslidarChDriver(const rclcpp::NodeOptions &options);

        ~LslidarChDriver();

        bool initialize();

        void publishLaserScan();

        void publishPointCloud();

        void packetProcessSingle(const lslidar_ch_driver::msg::LslidarPacket::UniquePtr &packet);
        
        void packetProcessDouble(const lslidar_ch_driver::msg::LslidarPacket::UniquePtr &packet);

        bool getLidarEcho(void);

        void convertCoordinate(Firing &data) {currentConverter(data);}

        void convertCoordinate_CX1S3(struct Firing &lidardata);

        void convertCoordinate_CX6S3(struct Firing &lidardata);

        void convertCoordinate_CH16X1(struct Firing &lidardata);

        void convertCoordinate_CH64W(struct Firing &lidardata);

        void convertCoordinate_CB64S1_A(struct Firing &lidardata);

        void convertCoordinate_CX126S3(struct Firing &lidardata);

        void convertCoordinate_128(struct Firing &lidardata); //ch128x1 ch128s1

        void convertCoordinate_CX128S2(struct Firing &lidardata);

        void convertCoordinate_CH256(struct Firing &lidardata);

        bool polling();

        void difopPoll(void);

        void initTimeStamp(void);

        void initializeMaxChannelNumbers(void);

        bool createRosIO();

        bool loadParameters();

        void outputParameters();

        inline void resetVariables_CH64W() {
            x = 0.0; y = 0.0; z = 0.0; sin_theat = 0.0; cos_theat = 0.0; _R_ = 0.0; cos_xita = 0.0; sin_xita = 0.0;
            cos_H_xita = 0.0; sin_H_xita = 0.0; cos_xita_F = 0.0; sin_xita_F = 0.0; add_distance = 0.0;
        }

        inline void resetVariables_CB64S1_A() {
            x = 0.0; y = 0.0; z = 0.0; sin_theat = 0.0; cos_theat = 0.0; _R_ = 0.0; cos_xita = 0.0; sin_xita = 0.0;
        }

        typedef std::shared_ptr<LslidarChDriver> LslidarChDriverPtr;
        typedef std::shared_ptr<const LslidarChDriver> LslidarChDriverConstPtr;

    private:
        //socket Parameters
        int msop_udp_port;
        int difop_udp_port;
        int echo_byte = 1211;

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
        bool publish_laserscan;
        bool gain_prism_angle;
        bool is_update_difop_packet;
        bool lidarEcho = false;
        std::string dump_file;
        double prism_angle[4];
        double prism_offset;
        double min_range;
        double max_range;
        double packet_rate;
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
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
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
        uint32_t packet_timestamp_s;
        uint32_t packet_timestamp_ns;
        double packet_timestamp;
        double last_packet_timestamp;
        double packet_interval_time;
        double point_interval_time;
        double point_cloud_timestamp;
        unsigned char packetTimeStamp[10];
        struct tm cur_time;

        double cx6s3_sin_theta_1[6];
        double cx6s3_sin_theta_2[6];
        double cx6s3_cos_theta_1[6];
        double cx6s3_cos_theta_2[6];

        double ch16x1_sin_theta_1[16];
        double ch16x1_sin_theta_2[16];
        double ch16x1_cos_theta_1[16];
        double ch16x1_cos_theta_2[16];

        double cx126s3_sin_theta_1[126];
        double cx126s3_sin_theta_2[126];
        double cx126s3_cos_theta_1[126];
        double cx126s3_cos_theta_2[126];

        double sin_theta_1[128];
        double sin_theta_2[128];
        double cos_theta_1[128];
        double cos_theta_2[128];

        double ch64w_sin_theta_1[128];
        double ch64w_sin_theta_2[128];
        double ch64w_cos_theta_1[128];
        double ch64w_cos_theta_2[128];

        double cb64s1_A_sin_theta_1[128];
        double cb64s1_A_sin_theta_2[128];
        double cb64s1_A_cos_theta_1[128];
        double cb64s1_A_cos_theta_2[128];

        double ch256_sin_theta_1[256];
        double ch256_sin_theta_2[256];
        double ch256_cos_theta_1[256];
        double ch256_cos_theta_2[256];

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
    };

    typedef LslidarChDriver::LslidarChDriverPtr LslidarChDriverPtr;
    typedef LslidarChDriver::LslidarChDriverConstPtr LslidarChDriverConstPtr;
    typedef PointXYZIRT VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;

} // namespace lslidar_ch_driver

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_ch_driver::PointXYZIRT,
                                                    (float, x, x)
                                                    (float, y, y)
                                                    (float, z, z)
                                                    (float, intensity, intensity)
                                                    (std::uint8_t, ring, ring)
                                                    (float, time, time))

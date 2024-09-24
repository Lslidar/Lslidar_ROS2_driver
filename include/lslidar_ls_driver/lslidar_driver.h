/******************************************************************************
 * This file is part of lslidar_ls_driver.
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

#include <regex>
#include <string>
#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
#include "input.h"
#include "lslidar_log.h"
#include <lslidar_ls_driver/msg/lslidar_ls_packet.hpp>
#include <lslidar_ls_driver/srv/angle_distortion_correction.hpp>
#include <lslidar_ls_driver/srv/data_ip.hpp>
#include <lslidar_ls_driver/srv/data_port.hpp>
#include <lslidar_ls_driver/srv/destination_ip.hpp>
#include <lslidar_ls_driver/srv/dev_port.hpp>
#include <lslidar_ls_driver/srv/frame_rate.hpp>
#include <lslidar_ls_driver/srv/invalid_data.hpp>
#include <lslidar_ls_driver/srv/standby_mode.hpp>
#include <lslidar_ls_driver/srv/time_service.hpp>
// #include <functional>
#include <chrono>
#include <deque>
#include <mutex>
#include "lslidar_ls_driver/ThreadPool.h"


namespace lslidar_driver {
    /** Special Defines for LS support **/
    constexpr int POINTS_PER_PACKET_SINGLE_ECHO = 1192;       // modify
    constexpr int POINTS_PER_PACKET_DOUBLE_ECHO = 1188;       // modify
    constexpr double SINGLE_ECHO = 0.006711409;
    constexpr double DOUBLE_ECHO = 0.01010101;

    static float g_fDistanceAcc = 0.001f;
    static float m_offset = 6.37f;
    constexpr double cos30 = cos(DEG2RAD(30));
    constexpr double sin30 = sin(DEG2RAD(30));
    //constexpr double cos60 = cos(DEG2RAD(60));
    constexpr double sin60 = sin(DEG2RAD(60));

    struct PointXYZIRT {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY
        uint16_t ring;
        float timestamp;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned

        PointXYZIRT(float x, float y, float z, float intensity, uint16_t ring, float timestamp)
        : x(x), y(y), z(z), intensity(intensity), ring(ring), timestamp(timestamp) {}

    } EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

    struct Firing {
        double vertical_angle;
        double azimuth;
        double distance;
        float intensity;
        double time;
        int channel_number;
    };

    class lslidarDriver : public rclcpp::Node {
    private:
        void difopPoll();

        void initTimeStamp();

        std::function<int(const struct Firing &)> lidarConvertCoordinate;

        int convertCoordinate(const struct Firing &lidardata);

        int convertCoordinateDistortion(const struct Firing &lidardata);

        std::function<void(const lslidar_ls_driver::msg::LslidarLsPacket::UniquePtr&)> lslidarPacketProcess;

        void packetProcessSingle(const lslidar_ls_driver::msg::LslidarLsPacket::UniquePtr& packet);

        void packetProcessDouble(const lslidar_ls_driver::msg::LslidarLsPacket::UniquePtr& packet);

        void checkPacketLoss(const lslidar_ls_driver::msg::LslidarLsPacket::UniquePtr &msg, int data_offset, int byte_count);

        // Publish data
        void publishPointCloud();

        bool loadParameters();

        void outputParameters();

        bool createRosIO();

        bool getLidarInformation();

        void pollThread(void);

        static void setPacketHeader(unsigned char *config_data);

        bool sendPacketTolidar(unsigned char *config_data) const;

        bool setAngleDistortionCorrection(std::shared_ptr <lslidar_ls_driver::srv::AngleDistortionCorrection::Request> req,
                                          std::shared_ptr <lslidar_ls_driver::srv::AngleDistortionCorrection::Response> res);

        bool setDataIp(std::shared_ptr<lslidar_ls_driver::srv::DataIp::Request> req,
                       std::shared_ptr<lslidar_ls_driver::srv::DataIp::Response> res);
        
        bool setDataPort(std::shared_ptr<lslidar_ls_driver::srv::DataPort::Request> req,
                         std::shared_ptr<lslidar_ls_driver::srv::DataPort::Response> res);

        bool setDestinationIp(std::shared_ptr<lslidar_ls_driver::srv::DestinationIp::Request> req,
                              std::shared_ptr<lslidar_ls_driver::srv::DestinationIp::Response> res);

        bool setDevPort(std::shared_ptr<lslidar_ls_driver::srv::DevPort::Request> req,
                        std::shared_ptr<lslidar_ls_driver::srv::DevPort::Response> res);
        
        bool setFrameRate(std::shared_ptr<lslidar_ls_driver::srv::FrameRate::Request> req,
                          std::shared_ptr<lslidar_ls_driver::srv::FrameRate::Response> res);
        
        bool setInvalidData(std::shared_ptr<lslidar_ls_driver::srv::InvalidData::Request> req,
                            std::shared_ptr<lslidar_ls_driver::srv::InvalidData::Response> res);
        
        bool setStandbyMode(std::shared_ptr<lslidar_ls_driver::srv::StandbyMode::Request> req,
                            std::shared_ptr<lslidar_ls_driver::srv::StandbyMode::Response> res);

        bool setTimeService(std::shared_ptr<lslidar_ls_driver::srv::TimeService::Request> req,
                            std::shared_ptr<lslidar_ls_driver::srv::TimeService::Response> res);

        unsigned char difop_data[1206];

        //socket Parameters
        int msop_udp_port{};
        int difop_udp_port{};

        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;

        // Converter convtor_
        std::shared_ptr<std::thread> difop_thread_;
        std::shared_ptr<std::thread> poll_thread_;

        // Ethernet relate variables
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string dump_file;

        in_addr lidar_ip{};
        int socket_id;
        bool add_multicast{};
        double prism_angle[4]{};

        // ROS related variables
        bool time_synchronization;
        uint64_t pointcloudTimeStamp{};
        unsigned char packetTimeStamp[10]{};
        rclcpp::Time timeStamp;

        // Configuration parameters
        double min_range;
        double max_range;
        
        rclcpp::Time packet_timeStamp;
        double packet_rate;
        double packet_end_time;
        double current_packet_time;
        double last_packet_time;
        double point_interval_time;
        double packet_interval_time;
        double point_cloud_timestamp;

        std::string pointcloud_topic;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_code_pub;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr packet_loss_pub_;
        
        rclcpp::Service<lslidar_ls_driver::srv::AngleDistortionCorrection>::SharedPtr angle_distortion_correction_service_;  //雷达内部角度畸变校正
        rclcpp::Service<lslidar_ls_driver::srv::DataIp>::SharedPtr data_ip_service_;                      // 数据包ip
        rclcpp::Service<lslidar_ls_driver::srv::DataPort>::SharedPtr data_port_service_;                  // 数据包端口
        rclcpp::Service<lslidar_ls_driver::srv::DestinationIp>::SharedPtr destination_ip_service_;        // 设备包ip
        rclcpp::Service<lslidar_ls_driver::srv::DevPort>::SharedPtr dev_port_service_;                    // 设备包端口
        rclcpp::Service<lslidar_ls_driver::srv::FrameRate>::SharedPtr frame_rate_service_;                // 帧率
        rclcpp::Service<lslidar_ls_driver::srv::InvalidData>::SharedPtr invalid_data_service_;            // 无效数据
        rclcpp::Service<lslidar_ls_driver::srv::StandbyMode>::SharedPtr standby_mode_service_;            // 待机模式
        rclcpp::Service<lslidar_ls_driver::srv::TimeService>::SharedPtr time_service_;                    // 时钟源
        
        int return_mode;
        int scan_start_angle{};
        int scan_end_angle{};
        double point_time;
        double g_fAngleAcc_V;
        std::mutex pc_lock;
        bool is_add_frame_;
        bool is_get_difop_;
        bool packet_loss;
        bool get_ms06_param;
        bool output_time_source;
        int64_t last_packet_number_;
        // int64_t current_packet_number_;
        int64_t tmp_packet_number_;
        int64_t total_packet_loss_;
        int frame_count;
        int m_horizontal_point = -1;

        double cos_table[36000]{};
        double sin_table[36000]{};
        double cos_mirror_angle[4]{};
        double sin_mirror_angle[4]{};
        std::unique_ptr <ThreadPool> threadPool_;

        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_pub_;

        typedef boost::shared_ptr<lslidarDriver> lslidarDriverPtr;
        typedef boost::shared_ptr<const lslidarDriver> lslidarDriverConstPtr;

    public:
        lslidarDriver();

        lslidarDriver(const rclcpp::NodeOptions &options);

        ~lslidarDriver() override;

        bool initialize();

        bool polling();
    };

    typedef PointXYZIRT VPoint;
}  // namespace lslidar_driver

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_driver::PointXYZIRT,
                                          (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (std::uint16_t, ring, ring)
                                          (float, timestamp, timestamp))

#endif

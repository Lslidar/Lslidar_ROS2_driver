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

#define M_PI 3.14159265358979323846

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include "input.h"
#include <lslidar_msgs/msg/lslidar_packet.hpp>
#include <lslidar_msgs/msg/lslidar_point.hpp>
#include <lslidar_msgs/msg/lslidar_scan.hpp>
#include <lslidar_msgs/srv/lslidarcontrol.hpp>
#include <chrono>
#include <deque>
#include <mutex>
#include "lslidar_driver/ThreadPool.h"


namespace lslidar_driver {
    /** Special Defines for LSCh support **/
    static const int POINTS_PER_PACKET_SINGLE_ECHO = 1192;       // modify
    static const int POINTS_PER_PACKET_DOUBLE_ECHO = 1188;       // modify
    static float g_fDistanceAcc = 0.001f;
    static double sin135 = sin(DEG2RAD(135));
    static double cos135 = cos(DEG2RAD(135));

    struct PointXYZIRT {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY
        uint16_t ring;
        double timestamp;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
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

        bool isPointInRange(const double &distance) const {
            return (distance >= min_range && distance <= max_range);
        }

        int convertCoordinate(const struct Firing &lidardata);

        // Publish data
        void publishPointCloud();

        void lslidarChPacketProcess(const lslidar_msgs::msg::LslidarPacket::UniquePtr &msg);

        bool loadParameters();

        bool createRosIO();

        //socket Parameters
        int msop_udp_port{};
        int difop_udp_port{};

        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;

        // Converter convtor_
        std::shared_ptr<std::thread> difop_thread_;

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
        double packet_end_time;
        double current_packet_time;
        double last_packet_time;
        double packet_rate{};
        double Angle_V_compensation;

        std::string pointcloud_topic;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr packet_loss_pub_;
        int return_mode;
        int scan_start_angle{};
        int scan_end_angle{};
        double point_time;
        double g_fAngleAcc_V;
        std::mutex pc_lock;
        bool is_add_frame_;
        bool is_get_difop_;
        uint64_t last_packet_number_;
        uint64_t current_packet_number_;
        uint64_t total_packet_loss_;
        int frame_count;

        double cos_table[36000]{};
        double sin_table[36000]{};
        double cos_mirror_angle[8]{};
        double sin_mirror_angle[8]{};
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
                                          (double, timestamp, timestamp))

#endif

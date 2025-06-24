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

#ifndef LSLIDAR_DRIVER_HPP
#define LSLIDAR_DRIVER_HPP

#define DEG_TO_RAD 0.017453f
#define RAD_TO_DEG 57.29577f

#include <rclcpp/rclcpp.hpp> 
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp> 
#include <sensor_msgs/msg/laser_scan.hpp> 

#include "input.hpp"
#include "ThreadPool.h"
#include "lslidar_device_info.hpp"
#include "lslidar_pointcloud.hpp"
#include "pointcloud_transform.hpp"
#include "lslidar_driver/lslidar_services.hpp"
#include "lslidar_msgs/msg/lslidar_information.hpp" 

namespace lslidar_driver {

    class LslidarDriver {
    public:
        explicit LslidarDriver(rclcpp::Node::SharedPtr node)
            : node_(node) {}
        
        virtual ~LslidarDriver() = default;

        virtual bool loadParameters() = 0;

        virtual void outputParameters() = 0;

        virtual bool createRosIO() = 0;

        virtual void initTimeStamp() = 0;

        virtual bool initialize() = 0;

        virtual bool poll() = 0;
        
    protected:
        rclcpp::Node::SharedPtr node_;
        
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub;
        rclcpp::Publisher<lslidar_msgs::msg::LslidarInformation>::SharedPtr lidar_info_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_pub;
        
        rclcpp::Service<lslidar_msgs::srv::IpAndPort>::SharedPtr network_config_service;
        rclcpp::Service<lslidar_msgs::srv::MotorSpeed>::SharedPtr motor_speed_service;
        rclcpp::Service<lslidar_msgs::srv::TimeMode>::SharedPtr time_mode_service;
        
        lslidar_msgs::msg::LslidarInformation::SharedPtr lidar_info_data;
        PointCloudTransform pointcloud_transform_;

        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;
        std::shared_ptr<LslidarServices> services_;
        std::shared_ptr<LidarDeviceInfo> device_info_ = std::make_shared<LidarDeviceInfo>();
        std::unique_ptr<ThreadPool> thread_pool_ = std::make_unique<ThreadPool>(3);

        std::atomic<bool> is_get_difop_{false};

        std::string lidar_type;
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string dump_file;
        std::string frame_id;
        std::string pointcloud_topic;
        
        bool add_multicast;
        bool use_time_service;
        bool is_pretreatment;

        int msop_udp_port;
        int difop_udp_port;

        double packet_rate;
        double min_range;
        double max_range;

        double x_offset;
        double y_offset;
        double z_offset;
        double roll;
        double pitch;
        double yaw;
    };

}  // namespace lslidar_driver

#endif  // LSLIDAR_DRIVER_HPP

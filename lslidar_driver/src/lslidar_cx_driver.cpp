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

#include "lslidar_driver/lslidar_cx_driver.hpp"
#include <std_msgs/msg/string.hpp> 
#include <sensor_msgs/point_cloud2_iterator.hpp>


namespace lslidar_driver {

    LslidarCxDriver::LslidarCxDriver(rclcpp::Node::SharedPtr node) : LslidarDriver(node),
                                                                     last_azimuth(0),
                                                                     sweep_end_time(0.0),
                                                                     is_first_sweep(true),
                                                                     return_mode(1),
                                                                     current_packet_time(0.0),
                                                                     last_packet_time(0.0),
                                                                     horizontal_angle_resolution(0.0),
                                                                     lidar_number_(1),
                                                                     config_vertical_angle_flag(0),
                                                                     point_cloud_xyzirt_(new pcl::PointCloud<VPoint>()),
                                                                     point_cloud_xyzi_(new pcl::PointCloud<pcl::PointXYZI>()),
                                                                     point_cloud_xyzirt_bak_(new pcl::PointCloud<VPoint>()),
                                                                     point_cloud_xyzi_bak_(new pcl::PointCloud<pcl::PointXYZI>()),
                                                                     scan_msg(new sensor_msgs::msg::LaserScan()),
                                                                     scan_msg_bak(new sensor_msgs::msg::LaserScan()),
                                                                     start_process_msop(false),
                                                                     is_msc16(true) {
        services_ = std::make_shared<LslidarCxServices>();
    }

    bool LslidarCxDriver::loadParameters() {
        node_->declare_parameter<std::string>("pcap", "");
        node_->declare_parameter<double>("packet_rate", 1695.0);
        node_->declare_parameter<std::string>("frame_id", "laser_link");
        node_->declare_parameter<bool>("add_multicast", false);
        node_->declare_parameter<std::string>("group_ip", "224.1.1.2");
        node_->declare_parameter<bool>("use_time_service", false);
        node_->declare_parameter<std::string>("device_ip", "");
        node_->declare_parameter<int>("msop_port", (int) MSOP_DATA_PORT_NUMBER);
        node_->declare_parameter<int>("difop_port", (int) DIFOP_DATA_PORT_NUMBER);
        node_->declare_parameter<std::string>("pointcloud_topic", "lslidar_point_cloud");
        node_->declare_parameter<bool>("use_first_point_time", false);
        node_->declare_parameter<bool>("use_absolute_time", false);
        node_->declare_parameter<double>("min_range", 0.3);
        node_->declare_parameter<double>("max_range", 150.0);
        node_->declare_parameter<bool>("is_pretreatment", false);
        node_->declare_parameter<double>("x_offset", 0.0);
        node_->declare_parameter<double>("y_offset", 0.0);
        node_->declare_parameter<double>("z_offset", 0.0);
        node_->declare_parameter<double>("roll", 0.0);
        node_->declare_parameter<double>("pitch", 0.0);
        node_->declare_parameter<double>("yaw", 0.0);
        node_->declare_parameter<bool>("read_once", false);
        node_->declare_parameter<bool>("read_fast", false);
        node_->declare_parameter<double>("repeat_delay", 0.0);

        node_->declare_parameter<std::string>("filter_angle_file", "");
        node_->declare_parameter<int>("angle_disable_min", 0);
        node_->declare_parameter<int>("angle_disable_max", 0);
        node_->declare_parameter<bool>("pcl_type", false);
        node_->declare_parameter<int>("scan_num", 0);
        node_->declare_parameter<double>("horizontal_angle_resolution", 0.2);
        node_->declare_parameter<bool>("publish_scan", false);

        node_->declare_parameter<bool>("is_MatrixTransformation", false);
        node_->declare_parameter<std::vector<double>>("transform_main", std::vector<double>());
        node_->declare_parameter<std::vector<double>>("transform_imu", std::vector<double>());

        node_->get_parameter("pcap", dump_file);
        node_->get_parameter("packet_rate", packet_rate);
        node_->get_parameter("frame_id", frame_id);
        node_->get_parameter("add_multicast", add_multicast);
        node_->get_parameter("group_ip", group_ip_string);
        node_->get_parameter("use_time_service", use_time_service);
        node_->get_parameter("device_ip", lidar_ip_string);
        node_->get_parameter("msop_port", msop_udp_port);
        node_->get_parameter("difop_port", difop_udp_port);
        node_->get_parameter("pointcloud_topic", pointcloud_topic);
        node_->get_parameter("use_first_point_time", use_first_point_time);
        node_->get_parameter("use_absolute_time", use_absolute_time);
        node_->get_parameter("min_range", min_range);
        node_->get_parameter("max_range", max_range);
        node_->get_parameter("is_pretreatment", is_pretreatment);
        node_->get_parameter("x_offset", x_offset);
        node_->get_parameter("y_offset", y_offset);
        node_->get_parameter("z_offset", z_offset);
        node_->get_parameter("roll", roll);
        node_->get_parameter("pitch", pitch);
        node_->get_parameter("yaw", yaw);

        node_->get_parameter("filter_angle_file", filter_angle_file);
        node_->get_parameter("angle_disable_min", angle_disable_min);
        node_->get_parameter("angle_disable_max", angle_disable_max);
        node_->get_parameter("scan_num", scan_num);
        node_->get_parameter("horizontal_angle_resolution", horizontal_angle_resolution);
        node_->get_parameter("publish_scan", publish_scan);

        // 获取转换矩阵
        node_->get_parameter("is_MatrixTransformation", is_MatrixTransformation);
        auto tTransform_main = node_->get_parameter("transform_main").as_double_array();
        auto tTransform_imu = node_->get_parameter("transform_imu").as_double_array();

        Eigen::Matrix4f MatrixTransform_main;
        Eigen::Matrix4f MatrixTransform_imu;

        for (int i = 0; i < 16; ++i) {
            if (!tTransform_main.empty()) MatrixTransform_main(i / 4, i % 4) = tTransform_main[i];
            if (!tTransform_imu.empty()) MatrixTransform_imu(i / 4, i % 4) = tTransform_imu[i];
        }

        if (!(tTransform_main.empty()) && (!tTransform_imu.empty()))
        {
            MatrixTransform_result = MatrixTransform_main * MatrixTransform_imu;
        }
        else if ((tTransform_main.empty()) && (!tTransform_imu.empty()))
        {
            MatrixTransform_result = MatrixTransform_imu;
        }
        else
        {
            MatrixTransform_result = MatrixTransform_main;
        }

        return true;
    }

    void LslidarCxDriver::outputParameters() {
        LS_PARAM << "pcap file: " << dump_file << LS_END;
        LS_PARAM << "packet rate: " << packet_rate << LS_END;
        LS_PARAM << "add multicast: " << std::boolalpha << add_multicast << LS_END;
        LS_PARAM << "frame id: " << frame_id << LS_END;
        LS_PARAM << "pointcloud topic: " << pointcloud_topic << LS_END;
        LS_PARAM << "use time service: " << std::boolalpha << use_time_service << LS_END;
        LS_PARAM << "use first point time: " << std::boolalpha << use_first_point_time << LS_END;
        LS_PARAM << "use absolute time: " << std::boolalpha << use_absolute_time << LS_END;
        LS_PARAM << "min range: " << min_range << LS_END;
        LS_PARAM << "max range: "<< max_range << LS_END;
        LS_PARAM << "angle disable min: " << angle_disable_min << LS_END;
        LS_PARAM << "angle disable max: " << angle_disable_max << LS_END;
        LS_PARAM << "publish scan: " << publish_scan << LS_END;
        LS_PARAM << "scan num: " << scan_num << LS_END;

        if (is_pretreatment) {
            LS_MSG << "x offset: " << x_offset << LS_END;
            LS_MSG << "y offset: " << y_offset << LS_END;
            LS_MSG << "z offset: " << z_offset << LS_END;
            LS_MSG << "roll: " << roll << LS_END;
            LS_MSG << "pitch: " << pitch << LS_END;
            LS_MSG << "yaw: " << yaw << LS_END;
        }

        LS_SOCKET << "Only accepting packets from IP address: " << lidar_ip_string.c_str() << LS_END;
        if (add_multicast) LS_SOCKET << "Opening UDP socket: group address: " << group_ip_string.c_str() << LS_END;
        LS_SOCKET << "Opening UDP socket msop port: " << msop_udp_port << LS_END;
        LS_SOCKET << "Opening UDP socket difop port: " << difop_udp_port << LS_END;
    }

    bool LslidarCxDriver::createRosIO() {
        pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10); 
        if (publish_scan) laserscan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        lidar_info_pub_ = node_->create_publisher<lslidar_msgs::msg::LslidarInformation>("lslidar_device_info", 1);
        time_pub_ = node_->create_publisher<std_msgs::msg::Float64>("time_topic", 10);

        network_config_service_ = node_->create_service<lslidar_msgs::srv::IpAndPort>("network_setup",
            [this](std::shared_ptr<lslidar_msgs::srv::IpAndPort::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::IpAndPort::Response> res) {
                std::dynamic_pointer_cast<LslidarCxServices>(services_)->setIpAndPort(req, res);
            });

        motor_speed_service_ = node_->create_service<lslidar_msgs::srv::MotorSpeed>("motor_speed",
            [this](std::shared_ptr<lslidar_msgs::srv::MotorSpeed::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::MotorSpeed::Response> res) {
                std::dynamic_pointer_cast<LslidarCxServices>(services_)->setMotorSpeed(req, res);
            });

        motor_control_service_ = node_->create_service<lslidar_msgs::srv::MotorControl>("motor_control",
            [this](std::shared_ptr<lslidar_msgs::srv::MotorControl::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::MotorControl::Response> res) {
                std::dynamic_pointer_cast<LslidarCxServices>(services_)->setMotorControl(req, res);
            });

        power_control_service_ = node_->create_service<lslidar_msgs::srv::PowerControl>("power_control",
            [this](std::shared_ptr<lslidar_msgs::srv::PowerControl::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::PowerControl::Response> res) {
                std::dynamic_pointer_cast<LslidarCxServices>(services_)->setPowerControl(req, res);
            });

        rfd_removal_service_ = node_->create_service<lslidar_msgs::srv::RfdRemoval>("remove_rain_fog_dust",
            [this](std::shared_ptr<lslidar_msgs::srv::RfdRemoval::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::RfdRemoval::Response> res) {
                std::dynamic_pointer_cast<LslidarCxServices>(services_)->setRfdRemoval(req, res);
            });

        tail_removal_service_ = node_->create_service<lslidar_msgs::srv::TailRemoval>("tail_remove",
            [this](std::shared_ptr<lslidar_msgs::srv::TailRemoval::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::TailRemoval::Response> res) {
                std::dynamic_pointer_cast<LslidarCxServices>(services_)->setTailRemoval(req, res);
            });

        time_mode_service_ = node_->create_service<lslidar_msgs::srv::TimeMode>("time_mode",
            [this](std::shared_ptr<lslidar_msgs::srv::TimeMode::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::TimeMode::Response> res) {
                std::dynamic_pointer_cast<LslidarCxServices>(services_)->setTimeMode(req, res);
            });

        if (!dump_file.empty()) {
            msop_input_.reset(new lslidar_driver::InputPCAP(node_, msop_udp_port, 1212, packet_rate, dump_file));
            difop_input_.reset(new lslidar_driver::InputPCAP(node_, difop_udp_port, 1206, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_driver::InputSocket(node_, msop_udp_port, 1212));
            difop_input_.reset(new lslidar_driver::InputSocket(node_, difop_udp_port, 1206));
        }
        
        thread_pool_->enqueue([this]() { difopPoll(); });

        return true;
    }

    void LslidarCxDriver::initTimeStamp() {
        for (int i = 0; i < 10; ++i) {
            this->packetTimeStamp[i] = 0;
        }
        this->packet_time_s = 0;
        this->packet_time_ns = 0;
        this->timeStamp = rclcpp::Time(0.0);

        point_time_offset = use_first_point_time ? 1 : 0;
        relative_time_offset = use_absolute_time ? 0 : 1;
    }

    bool LslidarCxDriver::initAngleConfig() {
        if (!filter_angle_file.empty()) {
            const std::string full_path = ament_index_cpp::get_package_share_directory("lslidar_driver") + "/" + filter_angle_file;
            YAML::Node filter_config;
            try {
                filter_config = YAML::LoadFile(full_path);
            } catch (const std::exception& e) { 
                LS_ERROR << "Failed to load yaml file: " << e.what() << LS_END;
                return false;
            }

            for (int i = 0; i < 32; i++) {
                filter_angle[i][0] = filter_config["angle"][i]["disable_min"].as<int>();
                filter_angle[i][1] = filter_config["angle"][i]["disable_max"].as<int>();

                while (filter_angle[i][0] < 0) filter_angle[i][0] += 36000;
                while (filter_angle[i][1] < 0) filter_angle[i][1] += 36000;
                while (filter_angle[i][0] > 36000) filter_angle[i][0] -= 36000;
                while (filter_angle[i][1] > 36000) filter_angle[i][1] -= 36000;
            }
        }

        while (angle_disable_min < 0) angle_disable_min += 36000;
        while (angle_disable_max < 0) angle_disable_max += 36000;            
        while (angle_disable_min > 36000) angle_disable_min -= 36000;            
        while (angle_disable_max > 36000) angle_disable_max -= 36000;

        if (angle_disable_max == angle_disable_min) {
            angle_able_min = 0;
            angle_able_max = 36000;
        } else {
            if (angle_disable_min < angle_disable_max && angle_disable_min != 0) {
                angle_able_min = angle_disable_max;
                angle_able_max = angle_disable_min + 36000;
            } else if (angle_disable_min < angle_disable_max && angle_disable_min == 0) {
                angle_able_min = angle_disable_max;
                angle_able_max = 36000;
            } else if (angle_disable_min > angle_disable_max) {
                angle_able_min = angle_disable_max;
                angle_able_max = angle_disable_min;
            }
        }

        for (int j = 0; j < 36000; ++j) {
            const float angle = static_cast<float>(j) * 0.01f * DEG_TO_RAD;
            sin_azimuth_table[j] = std::sin(angle); 
            cos_azimuth_table[j] = std::cos(angle);
        }

        return true;
    }

    bool LslidarCxDriver::initialize() {
        if (!loadParameters()) {
            LS_WARN << "Cannot load all required parameters" << LS_END;
            return false;
        }

        this->outputParameters();
        
        if (!createRosIO()) {
            LS_WARN << "Failed to create ROS2 interfaces" << LS_END;
            return false;
        }

        rclcpp::sleep_for(std::chrono::seconds(1));
        while (rclcpp::ok() && !determineLidarModel()) {
            LS_WARN << "Lidar model error, please check Lidar model. Retrying..." << LS_END;
    	    rclcpp::sleep_for(std::chrono::seconds(1));
        }
        
        if (!initAngleConfig()) {
            LS_WARN << "Failed to initialize angle configuration." << LS_END;
        }

        this->initTimeStamp();
        
        if (is_pretreatment) {
            pointcloud_transform_.setTransform(x_offset, y_offset, z_offset, roll, pitch, yaw);
        }
        
        point_cloud_xyzirt_->header.frame_id = frame_id;
        point_cloud_xyzirt_->height = 1;

        point_cloud_xyzi_->header.frame_id = frame_id;
        point_cloud_xyzi_->height = 1;

        if (publish_scan) {
            scan_msg->angle_min = -M_PI;
            scan_msg->angle_max = M_PI;
            scan_msg->range_min = min_range;
            scan_msg->range_max = max_range;
            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;

            point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::infinity());
            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        }

        return true;
    }

    void LslidarCxDriver::publishPointcloud() {
        if (!is_get_difop_.load()) return;
        std::unique_lock<std::mutex> lock(pointcloud_lock);
        
        auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        if (pcl_type) {
            if (point_cloud_xyzi_bak_->points.size() < 50) return;
            if (is_pretreatment) pointcloud_transform_.applyTransform(*point_cloud_xyzi_bak_);
            if (is_MatrixTransformation) pointcloud_transform_.applyTransform_2(*point_cloud_xyzi_bak_, MatrixTransform_result);

            pcl::toROSMsg(*point_cloud_xyzi_bak_, *pc_msg);
        } else {
            if (point_cloud_xyzirt_bak_->points.size() < 50) return;
            if (is_pretreatment) pointcloud_transform_.applyTransform(*point_cloud_xyzirt_bak_);
            if (is_MatrixTransformation) pointcloud_transform_.applyTransform_2(*point_cloud_xyzirt_bak_, MatrixTransform_result);
            
            pcl::toROSMsg(*point_cloud_xyzirt_bak_, *pc_msg);
        }

        pc_msg->header.stamp = rclcpp::Time(point_cloud_time * 1000000000LL); 
        pointcloud_pub_->publish(*pc_msg); 
        
        auto time_msg = std::make_shared<std_msgs::msg::Float64>();
        time_msg->data = point_cloud_time;
        time_pub_->publish(*time_msg);
    }

    void LslidarCxDriver::publishScan() {
        std::unique_lock<std::mutex> lock(pointcloud_lock);
        scan_msg_bak->header.frame_id = frame_id;
        scan_msg_bak->header.stamp = rclcpp::Time(point_cloud_time * 1000000000LL);
        laserscan_pub_->publish(std::move(scan_msg_bak));
    }

    bool LslidarCxDriver::checkPacketValidity(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) {
        for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            if (packet->data[blk_idx * 100] != 0xff && packet->data[blk_idx * 100 + 1] != 0xee) {
                return false;
            }
        }
        return true;
    }

    void LslidarCxDriver::decodePacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) {
        //couputer azimuth angle for each firing
        for (size_t b_idx = 0; b_idx < BLOCKS_PER_PACKET; ++b_idx) {
            firings.firing_azimuth[b_idx] = (packet->data[b_idx * 100 + 2] + (packet->data[b_idx * 100 + 3] << 8)) % 36000; //* 0.01 * DEG_TO_RAD;
        }
        for (size_t block_idx = 0; block_idx < BLOCKS_PER_PACKET; ++block_idx) {
            // computer distance ,intensity
            int32_t azimuth_diff_b = 0;
            if (return_mode == 1) {
                if (block_idx < BLOCKS_PER_PACKET - 1) {
                    azimuth_diff_b = firings.firing_azimuth[block_idx + 1] - firings.firing_azimuth[block_idx];
                    azimuth_diff_b = azimuth_diff_b < 0 ? azimuth_diff_b + 36000 : azimuth_diff_b;
                } else {
                    azimuth_diff_b = firings.firing_azimuth[block_idx] - firings.firing_azimuth[block_idx - 1];
                    azimuth_diff_b = azimuth_diff_b < 0 ? azimuth_diff_b + 36000 : azimuth_diff_b;
                }
            } else {
                //return mode 2
                if (block_idx < BLOCKS_PER_PACKET - 2) {
                    azimuth_diff_b = firings.firing_azimuth[block_idx + 2] - firings.firing_azimuth[block_idx];
                    azimuth_diff_b = azimuth_diff_b < 0 ? azimuth_diff_b + 36000 : azimuth_diff_b;
                } else {
                    azimuth_diff_b = firings.firing_azimuth[block_idx] - firings.firing_azimuth[block_idx - 2];
                    azimuth_diff_b = azimuth_diff_b < 0 ? azimuth_diff_b + 36000 : azimuth_diff_b;
                }
            }

            // 32 scan
            for (size_t scan_fir_idx = 0; scan_fir_idx < SCANS_PER_FIRING_CX; ++scan_fir_idx) {
                size_t byte_idx = RAW_SCAN_SIZE * scan_fir_idx;
                //azimuth
                firings.azimuth[block_idx * 32 + scan_fir_idx] = firings.firing_azimuth[block_idx] + scan_fir_idx * azimuth_diff_b / FIRING_TOFFSET;
                                                                 
                firings.azimuth[block_idx * 32 + scan_fir_idx] = firings.azimuth[block_idx * 32 + scan_fir_idx] % 36000;
                // distance
                firings.distance[block_idx * 32 + scan_fir_idx] = static_cast<float >((packet->data[block_idx * 100 + byte_idx + 4] + 
                                           (packet->data[block_idx * 100 + byte_idx + 5] << 8)) * DISTANCE_RESOLUTION * distance_unit);
                                            
                //intensity
                firings.intensity[block_idx * 32 + scan_fir_idx] = static_cast<float>(packet->data[block_idx * 100 + byte_idx + 6]);     
            }
        }

        return;
    }

    int LslidarCxDriver::calculateRing(size_t fir_idx) const {
        int remapped_scan_idx = 0;
        switch (ring_) {
            case 34:    // C32 5.0
                remapped_scan_idx = cx_v5_remap_table[fir_idx % 4] * 8 + (fir_idx % 32) / 8 + ((fir_idx % 8 >= 4) ? 4 : 0);
                break;
            case 33:    // CH32RN  C32WB
                remapped_scan_idx = (fir_idx % 32) % 4 * 16 >= 24 
                                  ? (fir_idx % 32) % 4 * 16 - 24 + fir_idx % 32 / 4 
                                  : (fir_idx % 32) % 4 * 16 + fir_idx % 32 / 4;
                break;
            case 32:
                remapped_scan_idx = (fir_idx % 32) % 4 * 8 + fir_idx % 32 / 4;
                break;
            case 31:    // C16 3.0
                remapped_scan_idx = (fir_idx % 32) % 2 == 0 ? (fir_idx % 32) / 2 : (fir_idx % 32) / 2 + 16;
                break;
            case 18:    // C16 5.0
                remapped_scan_idx = (fir_idx % 16) / 4 + cx_v5_remap_table[fir_idx % 4] * 4;
                break;
            case 17:    // C16 4.0 国产化
                remapped_scan_idx = (fir_idx % 16) / 4 + (fir_idx % 16) % 4 * 4;
                break;
            case 16:
                remapped_scan_idx = (fir_idx % 16) % 2 == 0  ? (fir_idx % 16) / 2 : (fir_idx % 16) / 2 + 8;
                break;
            case 8:
                remapped_scan_idx = (fir_idx % 8) % 2 * 4 + (fir_idx % 8) / 2;
                break;
            case 1:
                remapped_scan_idx = 0;
                break;
            default:
                remapped_scan_idx = 0;
                break;
        }

        return remapped_scan_idx;
    }

    bool LslidarCxDriver::isPointValid(const int fir_idx, const int remapped_scan_idx) const {
        if (!(firings.distance[fir_idx] >= min_range && firings.distance[fir_idx] <= max_range)) return false;

        if (angle_able_max > 36000) {
            if ((firings.azimuth[fir_idx] > (angle_able_max - 36000)) && firings.azimuth[fir_idx] < angle_able_min)  return false;
        } else {
            if ((firings.azimuth[fir_idx] > angle_able_max) || firings.azimuth[fir_idx] < angle_able_min) return false;
        }

        if (filter_angle[remapped_scan_idx][0] != 0 || filter_angle[remapped_scan_idx][1] != 0) {
            if (filter_angle[remapped_scan_idx][0] <= filter_angle[remapped_scan_idx][1]) {
                if (firings.azimuth[fir_idx] >= filter_angle[remapped_scan_idx][0] &&
                    firings.azimuth[fir_idx] <= filter_angle[remapped_scan_idx][1]) {
                    return false;
                }
            } else {
                if (firings.azimuth[fir_idx] >= filter_angle[remapped_scan_idx][0] ||
                    firings.azimuth[fir_idx] <= filter_angle[remapped_scan_idx][1]) {
                    return false;
                }
            }
        }

        return true;
    }

    bool LslidarCxDriver::poll() {
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet(new lslidar_msgs::msg::LslidarPacket());

        while (rclcpp::ok()) {
            // keep reading until full packet received
            int rc = msop_input_->getPacket(packet);
            if (rc >= 1206) break;       // got a full packet?
            if (rc == 0) continue;    // No full packet, retry
            if (rc < 0) return false; // System-level error, terminate,end of file reached?
        }

        //check if the packet is valid
        if (!checkPacketValidity(packet) || !start_process_msop)  return false;

        packet_num++;

        //decode the packet
        decodePacket(packet);
        // find the start of a new revolution
        // if there is one, new_sweep_start will be the index of the start firing,
        // otherwise, new_sweep_start will be FIRINGS_PER_PACKET.
        size_t new_sweep_start = 0;
        if (packet_num >= number_threshold) {
            do {
                if (abs(firings.azimuth[new_sweep_start] - last_azimuth) > 35900) {
                    packet_num = 0;
                    break;
                } else {
                    last_azimuth = firings.azimuth[new_sweep_start];
                    ++new_sweep_start;
                }
            } while (new_sweep_start < SCANS_PER_PACKET);
        } else {
            new_sweep_start = 384;
        }

        if (use_time_service &&  fpga_type >= 4) {
            if (time_service_mode == 1 || time_service_mode == 3 || time_service_mode == 4 || time_service_mode == 5) {    //ptp授时
                packet_time_s = packet->data[1201] * pow(2, 32) + (packet->data[1202] << 24) +
                               (packet->data[1203] << 16) + (packet->data[1204] << 8) + packet->data[1205];
                packet_time_ns = packet->data[1206] + (packet->data[1207] << 8) +
                                (packet->data[1208] << 16) + (packet->data[1209] << 24); //ns
                timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);// s,ns
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
            } else if (time_service_mode == 0) {          //gps授时
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_year = packet->data[1200] + 2000 - 1900;
                cur_time.tm_mon  = packet->data[1201] - 1;
                cur_time.tm_mday = packet->data[1202];
                cur_time.tm_hour = packet->data[1203];
                cur_time.tm_min  = packet->data[1204];
                cur_time.tm_sec  = packet->data[1205];
                packet_time_s  = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = packet->data[1206] + (packet->data[1207] << 8) +
                                (packet->data[1208] << 16) + (packet->data[1209] << 24); //ns
                timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
            } else if (time_service_mode == 2) {          //ntp授时
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_year = packet->data[1200] + 2000 - 1900;
                cur_time.tm_mon  = packet->data[1201] - 1;
                cur_time.tm_mday = packet->data[1202];
                cur_time.tm_hour = packet->data[1203];
                cur_time.tm_min  = packet->data[1204];
                cur_time.tm_sec  = packet->data[1205];
                packet_time_s  = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = (packet->data[1206] + (packet->data[1207] << 8) +
                                 (packet->data[1208] << 16) + (packet->data[1209] << 24)) * 1e3; //ns
                timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
            }
        } else if (use_time_service && fpga_type == 3) { // gps授时
            memset(&cur_time, 0, sizeof(cur_time));
            cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
            cur_time.tm_mon  = this->packetTimeStamp[8] - 1;
            cur_time.tm_mday = this->packetTimeStamp[7];
            cur_time.tm_hour = this->packetTimeStamp[6];
            cur_time.tm_min  = this->packetTimeStamp[5];
            cur_time.tm_sec  = this->packetTimeStamp[4];
            packet_time_s  = static_cast<uint64_t>(timegm(&cur_time)); //s
            packet_time_ns = (packet->data[1200] + packet->data[1201] * pow(2, 8) +
                              packet->data[1202] * pow(2, 16) + packet->data[1203] * pow(2, 24)) * 1e3; //ns
            timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);

            if ((timeStamp - timeStamp_bak).seconds() < 0.0 && (timeStamp - timeStamp_bak).seconds() > -1.0
                && packet_time_ns < 100000000) {
                timeStamp = rclcpp::Time(packet_time_s + 1, packet_time_ns);
            } else if ((timeStamp - timeStamp_bak).seconds() > 1.0 && (timeStamp - timeStamp_bak).seconds() < 1.2
                       && packet_time_ns > 900000000) {
                timeStamp = rclcpp::Time(packet_time_s - 1, packet_time_ns);
            }
            packet->stamp = timeStamp;
            current_packet_time = timeStamp.seconds();
            timeStamp_bak = timeStamp;
        } else {
            packet->stamp = node_->get_clock()->now();
            current_packet_time = node_->get_clock()->now().seconds();
        }

        size_t start_fir_idx = 0;
        size_t end_fir_idx = new_sweep_start;
        if (is_first_sweep && new_sweep_start == SCANS_PER_PACKET) {
            return true;
        } else {
            if (is_first_sweep) {
                is_first_sweep = false;
                start_fir_idx = new_sweep_start;
                end_fir_idx = SCANS_PER_PACKET;
            }
        }

        if (lidar_model == "c32_3") {
            for (int blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
                for (int scan_fir_idx = 0; scan_fir_idx < SCANS_PER_BLOCK; ++scan_fir_idx) {
                    if (1 >= scan_fir_idx % 4) {
                        firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                    } else {
                        firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                    }
                    
                    if (firings.azimuth[blk_idx * 32 + scan_fir_idx] < 0) {
                        firings.azimuth[blk_idx * 32 + scan_fir_idx] += 36000;
                    }
                    if (firings.azimuth[blk_idx * 32 + scan_fir_idx] > 36000) {
                        firings.azimuth[blk_idx * 32 + scan_fir_idx] -= 36000;
                    }
                }
            }
        }

        double point_interval_time = (current_packet_time - last_packet_time) * POINT_TIME_WEIGHT_CX;

        for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
            if (angle_change) {
                if ("C32W" == lidar_model) {
                    if (fir_idx % 32 == 29 || fir_idx % 32 == 6 || fir_idx % 32 == 14 || fir_idx % 32 == 22 ||
                        fir_idx % 32 == 30 || fir_idx % 32 == 7 || fir_idx % 32 == 15 || fir_idx % 32 == 23) {
                        firings.azimuth[fir_idx] += 389;
                    }
                    if (firings.azimuth[fir_idx] >= 36000) firings.azimuth[fir_idx] -= 36000;
                }
                if ("MSC16" == lidar_model) {
                    firings.azimuth[fir_idx] += msc16_offset_angle[fir_idx % 16] < 0 ?
                                                msc16_offset_angle[fir_idx % 16] + 36000 : msc16_offset_angle[fir_idx % 16];
                    if (firings.azimuth[fir_idx] >= 36000) { firings.azimuth[fir_idx] -= 36000; }
                }
            }
            
            int remapped_scan_idx = calculateRing(fir_idx);

            //check if the point is valid
            if (!isPointValid(fir_idx, remapped_scan_idx)) continue;

            //convert the point to xyz coordinate
            size_t table_idx = firings.azimuth[fir_idx];
            float cos_azimuth = cos_azimuth_table[table_idx];
            float sin_azimuth = sin_azimuth_table[table_idx];
            float x_coord, y_coord, z_coord;

            //Y-axis correspondence 0 degree
            int tmp_idx = firings.azimuth[fir_idx] - conversionAngle < 0 ?
                        firings.azimuth[fir_idx] - conversionAngle + 36000 : firings.azimuth[fir_idx] - conversionAngle;
                                                                            
            x_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * sin_azimuth +
                    R1 * sin_azimuth_table[tmp_idx];
            y_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * cos_azimuth +
                    R1 * cos_azimuth_table[tmp_idx];
            z_coord = firings.distance[fir_idx] * sin_scan_altitude[fir_idx % lidar_number_];
            
            // computer the time of the point
            double time;
            if (last_packet_time > 1e-6) {
                time = last_packet_time + point_interval_time * (fir_idx + 1) - sweep_end_time * relative_time_offset;
            } else {
                time = current_packet_time;
            }

            //add point
            if (pcl_type) {
                pcl::PointXYZI point_xyzi;
                point_xyzi.x = x_coord;
                point_xyzi.y = y_coord;
                point_xyzi.z = z_coord;
                point_xyzi.intensity = firings.intensity[fir_idx];
                point_cloud_xyzi_->points.push_back(point_xyzi);
                ++point_cloud_xyzi_->width;
            } else {
                VPoint point;
                point.time = time;
                point.x = x_coord;
                point.y = y_coord;
                point.z = z_coord;
                point.intensity = firings.intensity[fir_idx];
                point.ring = remapped_scan_idx;
                point_cloud_xyzirt_->points.push_back(point);
                ++point_cloud_xyzirt_->width;
            }
            
            if (publish_scan) {
                if (scan_num == remapped_scan_idx) {
                    float x = x_coord;
                    float y = y_coord;

                    // double range = sqrt(x_coord * x_coord + y_coord * y_coord + z_coord * z_coord);
                    double range = hypot(x, y);
                    double angle = atan2(y, x);

                    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
                    if (range < scan_msg->ranges[index]) {
                        scan_msg->ranges[index] = range;
                        scan_msg->intensities[index] = firings.intensity[fir_idx];
                    }
                }
            }
        }
        // a new sweep begins ----------------------------------------------------

        if (end_fir_idx != SCANS_PER_PACKET) {
            //publish Last frame scan
            if (last_packet_time > 1e-6) {
                sweep_end_time = last_packet_time + point_interval_time * (end_fir_idx + point_time_offset);
            } else {
                sweep_end_time = current_packet_time;
            }

            sweep_end_time = sweep_end_time > 0 ? sweep_end_time : 0;

            point_cloud_time = use_first_point_time ? last_point_cloud_time : sweep_end_time;
            last_point_cloud_time = sweep_end_time;

            {
                std::unique_lock<std::mutex> lock(pointcloud_lock);
                point_cloud_xyzirt_bak_ = std::move(point_cloud_xyzirt_);
                point_cloud_xyzi_bak_ = std::move(point_cloud_xyzi_);
                scan_msg_bak = std::move(scan_msg);
            }

            thread_pool_->enqueue([this]() { publishPointcloud(); });

            if (publish_scan) {
                thread_pool_->enqueue([this]() { publishScan(); });
            }

            point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
            point_cloud_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>);
            point_cloud_xyzirt_->header.frame_id = frame_id;
            point_cloud_xyzirt_->height = 1;

            point_cloud_xyzi_->header.frame_id = frame_id;
            point_cloud_xyzi_->height = 1;

            if (publish_scan) {
                scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
                scan_msg->angle_min = -M_PI;
                scan_msg->angle_max = M_PI;
                scan_msg->range_min = min_range;
                scan_msg->range_max = max_range;
                scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
                point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

                scan_msg->ranges.assign(point_size, std::numeric_limits<float>::infinity());
                scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
            }

            //prepare the next frame scan
            last_azimuth = firings.azimuth[SCANS_PER_PACKET - 1];
            start_fir_idx = end_fir_idx;
            end_fir_idx = SCANS_PER_PACKET;
            for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
                if (angle_change) {
                    if ("C32W" == lidar_model) {
                        if (fir_idx % 32 == 29 || fir_idx % 32 == 6 || fir_idx % 32 == 14 || fir_idx % 32 == 22 ||
                            fir_idx % 32 == 30 || fir_idx % 32 == 7 || fir_idx % 32 == 15 || fir_idx % 32 == 23) {
                            firings.azimuth[fir_idx] += 389;
                        }
                        if (firings.azimuth[fir_idx] >= 36000) firings.azimuth[fir_idx] -= 36000;
                    }
                    if ("MSC16" == lidar_model) {
                        firings.azimuth[fir_idx] += msc16_offset_angle[fir_idx % 16] < 0 ?
                                                    msc16_offset_angle[fir_idx % 16] + 36000 : msc16_offset_angle[fir_idx % 16];
                        if (firings.azimuth[fir_idx] >= 36000) { firings.azimuth[fir_idx] -= 36000; }
                    }
                }

                int remapped_scan_idx = calculateRing(fir_idx);

                //check if the point is valid
                if (!isPointValid(fir_idx, remapped_scan_idx)) continue;

                //convert the point to xyz coordinate
                size_t table_idx = firings.azimuth[fir_idx];
                float cos_azimuth = cos_azimuth_table[table_idx];
                float sin_azimuth = sin_azimuth_table[table_idx];
                float x_coord, y_coord, z_coord;

                //Y-axis correspondence 0 degree
                int tmp_idx = firings.azimuth[fir_idx] - conversionAngle < 0 ?
                            firings.azimuth[fir_idx] - conversionAngle + 36000 : firings.azimuth[fir_idx] - conversionAngle;
                                                                                
                x_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * sin_azimuth +
                        R1 * sin_azimuth_table[tmp_idx];
                y_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * cos_azimuth +
                        R1 * cos_azimuth_table[tmp_idx];
                z_coord = firings.distance[fir_idx] * sin_scan_altitude[fir_idx % lidar_number_];

                // computer the time of the point
                double time;
                if (last_packet_time > 1e-6) {
                    time = last_packet_time + point_interval_time * (fir_idx + 1) - sweep_end_time * relative_time_offset;
                } else {
                    time = current_packet_time;
                }

                //add point
                if (pcl_type) {
                    pcl::PointXYZI point_xyzi;
                    point_xyzi.x = x_coord;
                    point_xyzi.y = y_coord;
                    point_xyzi.z = z_coord;
                    point_xyzi.intensity = firings.intensity[fir_idx];
                    point_cloud_xyzi_->points.push_back(point_xyzi);
                    ++point_cloud_xyzi_->width;
                } else {
                    VPoint point;
                    point.time = time;
                    point.x = x_coord;
                    point.y = y_coord;
                    point.z = z_coord;
                    point.intensity = firings.intensity[fir_idx];
                    point.ring = remapped_scan_idx;
                    point_cloud_xyzirt_->points.push_back(point);
                    ++point_cloud_xyzirt_->width;
                }

                if (publish_scan) {
                    if (scan_num == remapped_scan_idx) {
                        float x = x_coord;
                        float y = y_coord;

                        // double range = sqrt(x_coord * x_coord + y_coord * y_coord + z_coord * z_coord);
                        double range = hypot(x, y);
                        double angle = atan2(y, x);

                        int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
                        if (range < scan_msg->ranges[index] && index < static_cast<int>(scan_msg->ranges.size())) {
                            scan_msg->ranges[index] = range;
                            scan_msg->intensities[index] = firings.intensity[fir_idx];
                        }
                    }
                }
            }
        }

        last_packet_time = current_packet_time;

        return true;
    }

    void LslidarCxDriver::difopPoll() {
        lslidar_msgs::msg::LslidarPacket::UniquePtr difop_packet(new lslidar_msgs::msg::LslidarPacket());
        static bool is_print_working_time = true;
        WorkingTime cxMapping;

        while (rclcpp::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc == 1206) {
                if (difop_packet->data[0] != 0xa5 || difop_packet->data[1] != 0xff ||
                    difop_packet->data[2] != 0x00 || difop_packet->data[3] != 0x5a) {
                    return;
                }

                // 服务配置雷达传递设备包
                services_->getDifopPacket(difop_packet);

                if (difop_packet->data[1196] == 0x03 || (difop_packet->data[1196] == 0x02 && difop_packet->data[1197] >> 4 == 8)) {
                    fpga_type = 3;
                    for (int i = 0; i < 32; ++i) {
                        uint8_t data1 = difop_packet->data[245 + 2 * i];
                        uint8_t data2 = difop_packet->data[245 + 2 * i + 1];
                        int vert_angle = data1 * 256 + data2;
                        vert_angle = vert_angle > 32767 ? (vert_angle - 65535) : vert_angle;
                        config_vertical_angle_32[i] = vert_angle * 0.01;
                    }

                    int angle_a0 = (difop_packet->data[186] << 8) + difop_packet->data[187];
                    adjust_angle[0] = angle_a0 > 32767 ? 32767 - angle_a0 : angle_a0;

                    int angle_a1 = (difop_packet->data[190] << 8) + difop_packet->data[191];
                    adjust_angle[1] = angle_a1 > 32767 ? 32767 - angle_a1 : angle_a1;

                    int angle_a2 = (difop_packet->data[188] << 8) + difop_packet->data[189];
                    adjust_angle[2] = angle_a2 > 32767 ? 32767 - angle_a2 : angle_a2;

                    int angle_a3 = (difop_packet->data[192] << 8) + difop_packet->data[193];
                    adjust_angle[3] = angle_a3 > 32767 ? 32767 - angle_a3 : angle_a3;

                    this->packetTimeStamp[4] = difop_packet->data[57];
                    this->packetTimeStamp[5] = difop_packet->data[56];
                    this->packetTimeStamp[6] = difop_packet->data[55];
                    this->packetTimeStamp[7] = difop_packet->data[54];
                    this->packetTimeStamp[8] = difop_packet->data[53];
                    this->packetTimeStamp[9] = difop_packet->data[52];
                } else if ((difop_packet->data[1202] >= 0x64) && (difop_packet->data[1203] >> 4) == 5){
                    fpga_type = 5;
                    time_service_mode = difop_packet->data[45];
                    remove_rain_fog_dust = difop_packet->data[49];
                    remove_mirror_points = difop_packet->data[50];;      // 去除玻璃镜像
                    remove_tail_points = difop_packet->data[51];;        // 去除拖尾档位
                    tail_filter_distance = (difop_packet->data[53] << 8) + difop_packet->data[54];      // 滤拖尾距离
             
                    if (is_print_working_time) { 
                        cxMapping = {
                            {156, 157, 158, 159},  // total_working_time
                            {160, 161, 162},       // less_minus40_pos
                            {163, 164, 165},       // minus40_to_minus10_pos
                            {166, 167, 168},       // minus10_to_30_pos
                            {169, 170, 171},       // positive30_to_70_pos
                            {172, 173, 174}        // positive70_to_100_pos
                        };

                        device_info_->parseAndPrintWorkingTime(difop_packet->data.data(), cxMapping);

                        if(use_time_service) LS_INFO << "The current lidar time synchronization mode is: " << time_service_mode << LS_END;
                        LS_INFO << "Remove rain, fog, and dust, level: " << remove_rain_fog_dust << LS_END;
                        LS_INFO << "Remove mirror points: " << (remove_mirror_points ? "ON" : "OFF") << LS_END;
                        LS_INFO << "Remove tail points, level: " << remove_tail_points << ", filter distance: " << tail_filter_distance << LS_END;

                        is_print_working_time = false;
                    }
                }  else if ((difop_packet->data[1198] & 0x0F) == 4){
                    fpga_type = 4;
                    time_service_mode = difop_packet->data[45];
                    remove_rain_fog_dust  = difop_packet->data[110];
                   
                    if (is_msc16 && difop_packet->data[1198] >> 4 == 7 && difop_packet->data[1202] >> 4 == 7) {
                        for (int j = 0; j < 16; ++j) {
                            int adjust_angle_ = (difop_packet->data[680 + 2 * j] << 8) + difop_packet->data[681 + 2 * j];
                            adjust_angle_ = adjust_angle_ > 32767 ? adjust_angle_ - 65535 : adjust_angle_;
                            msc16_adjust_angle[j] = static_cast<float>(adjust_angle_) * 0.01f;

                            int offset_angle = (difop_packet->data[712 + 2 * j] << 8) + difop_packet->data[713 + 2 * j];
                            offset_angle = offset_angle > 32767 ? offset_angle - 65535 : offset_angle;
                            msc16_offset_angle[j] = static_cast<float>(offset_angle) * 0.01f;
                            if (fabs(msc16_adjust_angle[j] - c16_vertical_angle[c16_remap_angle[j]]) > 1.5) {
                                config_vertical_angle_flag++;
                            }
                        }
                        if (config_vertical_angle_flag == 0) {
                            for (int k = 0; k < 16; ++k) {
                                c16_vertical_angle[c16_remap_angle[k]] = msc16_adjust_angle[k];
                            }
                        }
                        is_msc16 = false;
                    }

                    if (is_print_working_time) { 
                        cxMapping = {
                            {105, 106, 107, 108},  // total_working_time
                            {112, 113, 114},       // less_minus40_pos
                            {115, 116, 117},       // minus40_to_minus10_pos
                            {118, 119, 120},       // minus10_to_30_pos
                            {121, 122, 123},       // positive30_to_70_pos
                            {124, 125, 126}        // positive70_to_100_pos
                        };

                        device_info_->parseAndPrintWorkingTime(difop_packet->data.data(), cxMapping);
                        
                        if(use_time_service) LS_INFO << "The current lidar time synchronization mode is: " << time_service_mode << LS_END;
                        LS_INFO << "Remove rain, fog, and dust, level: " << remove_rain_fog_dust << LS_END;

                        is_print_working_time = false;
                    }
                }

                if (fpga_type != 0) {
                    Information device = device_info_->getCxDeviceInfo(difop_packet, fpga_type);

                    lidar_info_data_ = std::make_shared<lslidar_msgs::msg::LslidarInformation>();
                    lidar_info_data_->lidar_ip = device.lidarIp;
                    lidar_info_data_->destination_ip = device.destinationIP;
                    lidar_info_data_->lidar_mac_address = device.lidarMacAddress;
                    lidar_info_data_->msop_port = device.msopPort;
                    lidar_info_data_->difop_port = device.difopPort;
                    lidar_info_data_->lidar_serial_number = device.lidarSerialNumber;
                    lidar_info_data_->fpga_board_2_program = device.secondBoardProgram;
                    lidar_info_data_->fpga_board_3_program = device.thirdBoardProgram;
                    lidar_info_pub_->publish(*lidar_info_data_);
                }

                is_get_difop_.store(true);
            } else if (rc < 0) {
                return;
            }
        }
    }

    bool LslidarCxDriver::determineLidarModel(){
        if (!is_get_difop_.load()) return false;
        lslidar_msgs::msg::LslidarPacket::UniquePtr pkt(new lslidar_msgs::msg::LslidarPacket());
        
        while (rclcpp::ok()) {
            int rc_ = msop_input_->getPacket(pkt);
            if (rc_ >= 1206) break;
            if (rc_ == 0) continue;
            if (rc_ < 0) return false;
        }

        if (pkt->data[1211] == 0x01) {
            for (int i = 0; i < 1; ++i) {
                sin_scan_altitude[i] = sin(c1_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c1_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 1;
            lidar_number_ = 1;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_model = "C1";
            LS_INFO << "lidar model: C1" << LS_END;
            number_threshold = 1;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (3 == fpga_type && pkt->data[1211] == 0x10) {
            for (int j = 0; j < 16; ++j) {
                if (fabs(c16_30_vertical_angle[j] - config_vertical_angle_32[j]) > 1.5) {
                    config_vert_num++;
                }
            }
            if (config_vert_num == 0) {
                for (int k = 0; k < 16; ++k) {
                    sin_scan_altitude[k] = sin(config_vertical_angle_32[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(config_vertical_angle_32[k] * DEG_TO_RAD);
                }
            } else {
                for (int k = 0; k < 16; ++k) {
                    sin_scan_altitude[k] = sin(c16_30_vertical_angle[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(c16_30_vertical_angle[k] * DEG_TO_RAD);
                }
            }
            ring_ = 16;
            lidar_number_ = 16;
            R1 = R2_;
            conversionAngle = conversionAngle_C16_3;
            distance_unit = 0.25;
            lidar_model = "c16_3";
            LS_INFO << "lidar model: c16, version 3.0" << LS_END;
            if (pkt->data[1204] == 0x39) return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (3 == fpga_type && pkt->data[1211] == 0x20) {
            for (int j = 0; j < 32; ++j) {
                config_vertical_angle_tmp[j] = config_vertical_angle_32[adjust_angle_index[j]];

                if (fabs(c32_30_vertical_angle[j] - config_vertical_angle_tmp[j]) > 3.0) {
                    config_vert_num++;
                }
            }
            if (config_vert_num == 0) {
                for (int k = 0; k < 32; ++k) {
                    sin_scan_altitude[k] = sin(config_vertical_angle_tmp[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(config_vertical_angle_tmp[k] * DEG_TO_RAD);
                }
            } else {
                for (int k = 0; k < 32; ++k) {
                    sin_scan_altitude[k] = sin(c32_30_vertical_angle[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(c32_30_vertical_angle[k] * DEG_TO_RAD);
                }
            }
            ring_ = 31;
            lidar_number_ = 32;
            R1 = R3_;
            conversionAngle = conversionAngle_C32_3;
            lidar_model = "c32_3";
            LS_INFO << "lidar model: c32, version 3.0 " << LS_END;
            if (pkt->data[1204] == 0x39) return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x03) {
            for (int i = 0; i < 1; ++i) {
                sin_scan_altitude[i] = sin(c1_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c1_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 1;
            lidar_number_ = 1;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_model = "C1P";
            LS_INFO << "lidar model: C1P" << LS_END;
            number_threshold = 1;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x06){
            for (int i = 0; i < 1; ++i) {
                sin_scan_altitude[i] = sin(c1_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c1_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 1;
            lidar_number_ = 1;
            distance_unit = 0.1;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_model = "N301";
            LS_INFO << "lidar model: N301" << LS_END;
            number_threshold = 1;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x07) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c8f_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c8f_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 8;
            lidar_number_ = 8;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_model = "C8F";         
            LS_INFO << "lidar model: C8F" << LS_END;
            number_threshold = 2;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x08) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c8_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c8_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 8;
            lidar_number_ = 8;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_model = "c8";               
            LS_INFO << "lidar model: C8" << LS_END;
            number_threshold = 2;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x09) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(ckm8_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(ckm8_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 8;
            lidar_number_ = 8;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_model = "CKM8";
            LS_INFO << "lidar model: CKM8/C4" << LS_END;
            number_threshold = 2;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x10) {
            for (int i = 0; i < 16; ++i) {
                sin_scan_altitude[i] = sin(c16_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c16_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 16;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_number_ = 16;
            lidar_model = "C16";
            LS_INFO << "lidar model: C16" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x65) {
            for (int i = 0; i < 16; ++i) {
                sin_scan_altitude[i] = sin(c16_v5_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c16_v5_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 18;
            R1 = R1_v5;
            conversionAngle = conversionAngle_v5;
            lidar_number_ = 16;
            lidar_model = "C16";
            LS_INFO << "lidar model: C16 v5" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x11) {
            for (int i = 0; i < 16; ++i) {
                sin_scan_altitude[i] = sin(c16_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c16_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 16;
            lidar_number_ = 16;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            angle_change = true;  
            lidar_model = "MSC16";        
            LS_INFO << "lidar model: MSC16" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x12) {
            for (int i = 0; i < 16; ++i) {
                sin_scan_altitude[i] = sin(c16_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c16_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 17;
            lidar_number_ = 16;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_model = "C16_domestic";
            LS_INFO << "lidar model: C16_domestic" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x20) {
            for (int i = 0; i < 32; ++i) {
                sin_scan_altitude[i] = sin(c32_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c32_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 32;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            LS_INFO << "lidar model: C32" << LS_END;
            lidar_number_ = 32;
            lidar_model = "C32";
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x64) {
            for (int i = 0; i < 32; ++i) {
                sin_scan_altitude[i] = sin(c32_v5_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c32_v5_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 34;
            R1 = R1_v5;
            conversionAngle = conversionAngle_v5;
            lidar_number_ = 32;
            lidar_model = "C32";
            LS_INFO << "lidar model: C32 v5" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x45) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32wp_vertical_angle[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32wp_vertical_angle[k] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_C32W;
            conversionAngle = conversionAngle_;                
            lidar_model = "C32WP";
            LS_INFO << "lidar model: C32WP" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x46) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32_70_vertical_angle[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32_70_vertical_angle[k] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_C32W;
            angle_change = true;
            conversionAngle = conversionAngle_;
            lidar_model = "C32W";
            LS_INFO << "lidar model: C32W" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x47) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32wn_vertical_angle2[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32wn_vertical_angle2[k] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_C32W;
            conversionAngle = conversionAngle_;
            lidar_model = "C32WN";
            LS_INFO << "lidar model: C32WN" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x48) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32wb_vertical_angle2[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32wb_vertical_angle2[k] * DEG_TO_RAD);
            }
            ring_ = 33;
            lidar_number_ = 32;
            R1 = R1_C32W;
            conversionAngle = conversionAngle_;
            lidar_model = "C32WB";
            LS_INFO << "lidar model: C32WB" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x5a) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32_90_vertical_angle[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32_90_vertical_angle[k] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_90;
            conversionAngle = conversionAngle_90;
            lidar_model = "CH32R";
            LS_INFO << "lidar model: CH32R" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x5d) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32rn_vertical_angle[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32rn_vertical_angle[k] * DEG_TO_RAD);
            }
            ring_ = 33;
            lidar_number_ = 32;
            R1 = R1_90;
            conversionAngle = conversionAngle_90;
            lidar_model = "CH32RN";
            LS_INFO << "lidar model: CH32R" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x00 && pkt->data[1205] == 0x10) {
            for (int j = 0; j < 16; ++j) {
                if (fabs(c16_30_vertical_angle[j] - config_vertical_angle_32[j]) > 1.5) {
                    config_vert_num++;
                }
            }
            if (config_vert_num == 0) {
                for (int k = 0; k < 16; ++k) {
                    sin_scan_altitude[k] = sin(config_vertical_angle_32[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(config_vertical_angle_32[k] * DEG_TO_RAD);
                }
            } else {
                for (int k = 0; k < 16; ++k) {
                    sin_scan_altitude[k] = sin(c16_30_vertical_angle[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(c16_30_vertical_angle[k] * DEG_TO_RAD);
                }
            }
            ring_ = 16;
            lidar_number_ = 16;
            R1 = R2_;
            conversionAngle = conversionAngle_C16_3;
            distance_unit = 0.25;
            lidar_model = "c16_3";
            LS_INFO << "lidar model: c16, version 3.0" << LS_END;
            if (pkt->data[1204] == 0x39) return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else if (pkt->data[1211] == 0x00 && pkt->data[1205] == 0x20) {
            for (int j = 0; j < 32; ++j) {
                config_vertical_angle_tmp[j] = config_vertical_angle_32[adjust_angle_index[j]];

                if (fabs(c32_30_vertical_angle[j] - config_vertical_angle_tmp[j]) > 3.0) {
                    config_vert_num++;
                }
            }
            if (config_vert_num == 0) {
                for (int k = 0; k < 32; ++k) {
                    sin_scan_altitude[k] = sin(config_vertical_angle_tmp[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(config_vertical_angle_tmp[k] * DEG_TO_RAD);
                }
            } else {
                for (int k = 0; k < 32; ++k) {
                    sin_scan_altitude[k] = sin(c32_30_vertical_angle[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(c32_30_vertical_angle[k] * DEG_TO_RAD);
                }
            }
            ring_ = 31;
            lidar_number_ = 32;
            R1 = R3_;
            conversionAngle = conversionAngle_C32_3;
            lidar_model = "c32_3";
            LS_INFO << "lidar model: c32, version 3.0 " << LS_END;
            if (pkt->data[1204] == 0x39) return_mode = 2;
            LS_INFO << "return mode: " << return_mode << LS_END;
        } else {
            return false;
        }

        start_process_msop = true;
        return true;
    }
}  // namespace lslidar_driver

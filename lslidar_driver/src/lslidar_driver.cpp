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

#include "lslidar_driver/lslidar_driver.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <functional>

#include <memory>

namespace lslidar_driver {
    LslidarDriver::LslidarDriver() : LslidarDriver(rclcpp::NodeOptions()) {
        return;
    }

    LslidarDriver::LslidarDriver(const rclcpp::NodeOptions &options) : Node("lslidar_node", options) {
        last_azimuth = 0;
        sweep_end_time = 0.0;
        is_first_sweep = true,
        return_mode = 1,
        packet_rate = 1695.0,
        current_packet_time = 0.0;
        last_packet_time = 0.0;
        current_point_time = 0.0,
        last_point_time = 0.0,
        horizontal_angle_resolution = 0.0,
        lidar_number_ = 1,
        point_num = 2000,
        sweep_data = std::make_unique<lslidar_msgs::msg::LslidarScan>(),
        sweep_data_bak = std::make_unique<lslidar_msgs::msg::LslidarScan>();

        return;
    }

    bool LslidarDriver::checkPacketValidity(const lslidar_driver::RawPacket *packet) {
        for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            if (packet->blocks[blk_idx].header != UPPER_BANK) {
                return false;
            }
        }
        return true;
    }

    bool LslidarDriver::isPointInRange(const double &distance) {
        return (distance >= min_range && distance < max_range);
    }

    bool LslidarDriver::loadParameters() {
        this->declare_parameter<std::string>("pcap", "");
        this->declare_parameter<double>("packet_rate", 1695.0);
        this->declare_parameter<std::string>("device_ip", "192.168.1.200");
        this->declare_parameter<int>("msop_port", 2368);
        this->declare_parameter<int>("difop_port", 2369);
        this->declare_parameter<bool>("pcl_type", false);
        this->declare_parameter<std::string>("c32_type", "c32_32");
        this->declare_parameter<std::string>("lidar_type", "c16");
        this->declare_parameter<bool>("add_multicast", false);
        this->declare_parameter<std::string>("group_ip", "234.2.3.2");
        this->declare_parameter<bool>("use_gps_ts", false);
        this->declare_parameter<double>("min_range", 0.3);
        this->declare_parameter<double>("max_range", 150.0);
        this->declare_parameter<std::string>("frame_id", "laser_link");
        this->declare_parameter<double>("distance_unit", 0.25);
        this->declare_parameter<int>("angle_disable_min", 0);
        this->declare_parameter<int>("angle_disable_max", 0);
        this->declare_parameter<double>("horizontal_angle_resolution", 0.18);
        this->declare_parameter<int>("scan_num", 15);
        this->declare_parameter<bool>("publish_scan", false);
        this->declare_parameter<std::string>("topic_name", "lslidar_point_cloud");
        this->declare_parameter<bool>("coordinate_opt", false);

        msop_udp_port = 0;
        difop_udp_port = 0;
        this->get_parameter("pcap", dump_file);
        this->get_parameter("packet_rate", packet_rate);
        this->get_parameter("device_ip", lidar_ip_string);
        this->get_parameter("msop_port", msop_udp_port);
        this->get_parameter("difop_port", difop_udp_port);
        this->get_parameter("pcl_type", pcl_type);
        this->get_parameter("lidar_type", lidar_type);
        this->get_parameter("c32_type", c32_type);
        this->get_parameter("add_multicast", add_multicast);
        this->get_parameter("group_ip", group_ip_string);
        this->get_parameter("use_gps_ts", use_gps_ts);
        this->get_parameter("min_range", min_range);
        this->get_parameter("max_range", max_range);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("distance_unit", distance_unit);
        this->get_parameter("angle_disable_min", angle_disable_min);
        this->get_parameter("angle_disable_max", angle_disable_max);
        this->get_parameter("horizontal_angle_resolution", horizontal_angle_resolution);
        this->get_parameter("scan_num", scan_num);
        this->get_parameter("publish_scan", publish_scan);
        this->get_parameter("topic_name", pointcloud_topic);
        this->get_parameter("coordinate_opt", coordinate_opt);

        RCLCPP_INFO(this->get_logger(), "dump_file: %s", dump_file.c_str());
        RCLCPP_INFO(this->get_logger(), "packet_rate: %f", packet_rate);
        RCLCPP_INFO(this->get_logger(), "device_ip: %s", lidar_ip_string.c_str());
        RCLCPP_INFO(this->get_logger(), "msop_port: %d", msop_udp_port);
        RCLCPP_INFO(this->get_logger(), "difop_udp_port: %d", difop_udp_port);
        RCLCPP_INFO(this->get_logger(), "pcl_type: %d", pcl_type);
        RCLCPP_INFO(this->get_logger(), "lidar_type: %s", lidar_type.c_str());
        RCLCPP_INFO(this->get_logger(), "c32_type: %s", c32_type.c_str());
        RCLCPP_INFO(this->get_logger(), "add_multicast: %d", add_multicast);
        RCLCPP_INFO(this->get_logger(), "group_ip_string: %s", group_ip_string.c_str());
        RCLCPP_INFO(this->get_logger(), "use_gps_ts: %d", use_gps_ts);
        RCLCPP_INFO(this->get_logger(), "min_range: %f", min_range);
        RCLCPP_INFO(this->get_logger(), "max_range: %f", max_range);
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "distance_unit: %f", distance_unit);
        RCLCPP_INFO(this->get_logger(), "angle_disable_min: %d", angle_disable_min);
        RCLCPP_INFO(this->get_logger(), "angle_disable_max: %d", angle_disable_max);
        RCLCPP_INFO(this->get_logger(), "horizontal_angle_resolution: %f", horizontal_angle_resolution);
        RCLCPP_INFO(this->get_logger(), "scan_num: %d", scan_num);
        RCLCPP_INFO(this->get_logger(), "publish_scan: %d", publish_scan);
        RCLCPP_INFO(this->get_logger(), "topic_name: %s", pointcloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "coordinate_opt: %d", coordinate_opt);

        if (lidar_type == "c16") {
            if (scan_num > 15) { scan_num = 15; }
            else if (scan_num < 0) { scan_num = 0; }
        } else if (lidar_type == "c8") {
            if (scan_num > 7) { scan_num = 7; }
            else if (scan_num < 0) { scan_num = 0; }
        } else if (lidar_type == "c1") {
            scan_num = 0;
            publish_scan = true;
        } else if (lidar_type == "c32") {
            if (scan_num > 31) { scan_num = 31; }
            else if (scan_num < 0) { scan_num = 0; }
        }


        inet_aton(lidar_ip_string.c_str(), &lidar_ip);
        if (add_multicast)
            RCLCPP_INFO(this->get_logger(), "opening UDP socket: group_address %s", group_ip_string.c_str());
        return true;
    }

    void LslidarDriver::initTimeStamp() {
        for (int i = 0; i < 10; i++) {
            this->packetTimeStamp[i] = 0;
        }
        this->packet_time_s = 0;
        this->packet_time_ns = 0;
        this->timeStamp = rclcpp::Time(0.0);
    }

    bool LslidarDriver::createRosIO() {
        pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        lslidar_control_service_ = this->create_service<lslidar_msgs::srv::LslidarControl>("lslidar_control",
                                                                                           std::bind(
                                                                                                   &LslidarDriver::powerOn,
                                                                                                   this,
                                                                                                   std::placeholders::_1,
                                                                                                   std::placeholders::_2),
                                                                                           qos_profile);
        motor_control_service_ = this->create_service<lslidar_msgs::srv::MotorControl>("motor_control",
                                                                                           std::bind(
                                                                                                   &LslidarDriver::motorControl,
                                                                                                   this,
                                                                                                   std::placeholders::_1,
                                                                                                   std::placeholders::_2),
                                                                                           qos_profile);
        motor_speed_service_ = this->create_service<lslidar_msgs::srv::MotorSpeed>("set_motor_speed",
                                                                                           std::bind(
                                                                                                   &LslidarDriver::motorSpeed,
                                                                                                   this,
                                                                                                   std::placeholders::_1,
                                                                                                   std::placeholders::_2),
                                                                                           qos_profile);
        data_port_service_ = this->create_service<lslidar_msgs::srv::DataPort>("set_data_port",
                                                                                           std::bind(
                                                                                                   &LslidarDriver::setDataPort,
                                                                                                   this,
                                                                                                   std::placeholders::_1,
                                                                                                   std::placeholders::_2),
                                                                                           qos_profile);
        dev_port_service_ = this->create_service<lslidar_msgs::srv::DevPort>("set_dev_port",
                                                                                           std::bind(
                                                                                                   &LslidarDriver::setDevPort,
                                                                                                   this,
                                                                                                   std::placeholders::_1,
                                                                                                   std::placeholders::_2),
                                                                                           qos_profile);
        data_ip_service_ = this->create_service<lslidar_msgs::srv::DataIp>("set_data_ip",
                                                                                           std::bind(
                                                                                                   &LslidarDriver::setDataIp,
                                                                                                   this,
                                                                                                   std::placeholders::_1,
                                                                                                   std::placeholders::_2),
                                                                                           qos_profile);
        destination_ip_service_ = this->create_service<lslidar_msgs::srv::DestinationIp>("set_destination_ip",
                                                                                           std::bind(
                                                                                                   &LslidarDriver::setDestinationIp,
                                                                                                   this,
                                                                                                   std::placeholders::_1,
                                                                                                   std::placeholders::_2),
                                                                                           qos_profile);
        time_service_ = this->create_service<lslidar_msgs::srv::TimeService>("time_service",
                                                                                           std::bind(
                                                                                                   &LslidarDriver::timeService,
                                                                                                   this,
                                                                                                   std::placeholders::_1,
                                                                                                   std::placeholders::_2),
                                                                                           qos_profile);

        if (dump_file != "") {
            msop_input_.reset(new lslidar_driver::InputPCAP(this, msop_udp_port, 1212, packet_rate, dump_file));
            difop_input_.reset(new lslidar_driver::InputPCAP(this, difop_udp_port, 1206, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_driver::InputSocket(this, msop_udp_port, 1212));
            difop_input_.reset(new lslidar_driver::InputSocket(this, difop_udp_port, 1206));
        }
        difop_thread_ = std::shared_ptr<std::thread>(
                new std::thread(std::bind(&LslidarDriver::difopPoll, this)));

        poll_thread_ = std::shared_ptr<std::thread>(
                new std::thread(std::bind(&LslidarDriver::pollThread, this)));
        return true;
    }

    bool LslidarDriver::initialize() {
        this->initTimeStamp();
        if (!loadParameters()) {
            RCLCPP_INFO(this->get_logger(), "cannot load all required ROS parameters.");
            return false;
        }
        if (!createRosIO()) {
            RCLCPP_INFO(this->get_logger(), "cannot create all ROS IO.");
            return false;
        }

        if (lidar_type == "c16") {
            lidar_number_ = 16;
            RCLCPP_INFO(this->get_logger(), "lidar: c16");
            for (int i = 0; i < 16; ++i) {
                sin_scan_altitude[i] = sin(c16_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c16_vertical_angle[i] * DEG_TO_RAD);
            }
        } else if (lidar_type == "c8") {
            lidar_number_ = 8;
            RCLCPP_INFO(this->get_logger(), "lidar: c8");
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c8_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c8_vertical_angle[i] * DEG_TO_RAD);
            }
        } else if (lidar_type == "c1") {
            lidar_number_ = 1;
            RCLCPP_INFO(this->get_logger(), "lidar: c1");
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c1_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c1_vertical_angle[i] * DEG_TO_RAD);
            }
        } else if (lidar_type == "c32") {
            lidar_number_ = 32;
            if ("c32_32" == c32_type) {
                RCLCPP_INFO(this->get_logger(), "Vertical angle: 32 degrees");
                for (int i = 0; i < 32; ++i) {
                    sin_scan_altitude[i] = sin(c32_vertical_angle[i] * DEG_TO_RAD);
                    cos_scan_altitude[i] = cos(c32_vertical_angle[i] * DEG_TO_RAD);
                }
            } else if ("c32_70" == c32_type) {
                RCLCPP_INFO(this->get_logger(), "Vertical angle: 70 degrees");
                for (int k = 0; k < 32; ++k) {
                    sin_scan_altitude[k] = sin(c32_70_vertical_angle[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(c32_70_vertical_angle[k] * DEG_TO_RAD);
                }
            } else if ("c32_90" == c32_type) {
                RCLCPP_INFO(this->get_logger(), "Vertical angle: 90 degrees");
                for (int k = 0; k < 32; ++k) {
                    sin_scan_altitude[k] = sin(c32_90_vertical_angle[k] * DEG_TO_RAD);
                    cos_scan_altitude[k] = cos(c32_90_vertical_angle[k] * DEG_TO_RAD);
                }
            }
        }

        // create the sin and cos table for different azimuth values
        for (int j = 0; j < 36000; ++j) {
            double angle = static_cast<double>(j) / 100.0 * DEG_TO_RAD;
            sin_azimuth_table[j] = sin(angle);
            cos_azimuth_table[j] = cos(angle);
        }
        return true;
    }

    void LslidarDriver::difopPoll() {
        // reading and publishing scans as fast as possible.
        lslidar_msgs::msg::LslidarPacket::UniquePtr difop_packet_ptr(new lslidar_msgs::msg::LslidarPacket());
        while (rclcpp::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet_ptr);
            if (rc == 0) {
                if (difop_packet_ptr->data[0] != 0xa5 || difop_packet_ptr->data[1] != 0xff ||
                    difop_packet_ptr->data[2] != 0x00 || difop_packet_ptr->data[3] != 0x5a) {
                    return;
                }
                time_service_mode_ = difop_packet_ptr->data[45];
                for (int i = 0; i < 1206; i++) {
                    difop_data[i] = difop_packet_ptr->data[i];
                }
                is_get_difop_ = true;
            } else if (rc < 0) {
                return;
            }
        }
    }

    void LslidarDriver::pointcloudToLaserscan(const sensor_msgs::msg::PointCloud2 &cloud_msg,
                                              sensor_msgs::msg::LaserScan &output_scan) {
        // build laserscan output_scan
        output_scan.header = cloud_msg.header;
        output_scan.header.frame_id = cloud_msg.header.frame_id;
        output_scan.angle_min = -M_PI;
        output_scan.angle_max = M_PI;
        output_scan.angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
        output_scan.time_increment = 0.0;
//        output_scan.scan_time = scan_time_;
        output_scan.range_min = min_range;
        output_scan.range_max = max_range;

        // determine amount of rays to create
        uint32_t ranges_size = std::ceil((output_scan.angle_max - output_scan.angle_min) / output_scan.angle_increment);

        // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
        output_scan.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
		output_scan.intensities.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

        // Iterate through pointcloud
        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x"), iter_y(cloud_msg, "y"),
                     iter_z(cloud_msg, "z"), iter_intensity(cloud_msg, "intensity");
             iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
                RCLCPP_DEBUG(this->get_logger(), "rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
                continue;
            }

            double range = hypot(*iter_x, *iter_y);
            if (range < min_range) {
                RCLCPP_DEBUG(this->get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
                             range, min_range,
                             *iter_x,
                             *iter_y, *iter_z);
                continue;
            }
            if (range > max_range) {
                RCLCPP_DEBUG(this->get_logger(), "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
                             range, max_range,
                             *iter_x,
                             *iter_y, *iter_z);
                continue;
            }

            double angle = atan2(*iter_y, *iter_x);
            if (angle < output_scan.angle_min || angle > output_scan.angle_max) {
                RCLCPP_DEBUG(this->get_logger(), "rejected for angle %f not in range (%f, %f)\n", angle,
                             output_scan.angle_min,
                             output_scan.angle_max);
                continue;
            }

            // overwrite range at laserscan ray if new range is smaller
            int index = (angle - output_scan.angle_min) / output_scan.angle_increment;
            if (range < output_scan.ranges[index]) {
                output_scan.ranges[index] = range;
                output_scan.intensities[index] = *iter_intensity;
            }
        }
    }

    void LslidarDriver::publishPointcloud() {
		if (sweep_data_bak->points.size() < 65) return;
        std::unique_lock <std::mutex> lock(pointcloud_lock);
        if (pcl_type) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud <pcl::PointXYZI>);
            point_cloud->header.frame_id = frame_id;
            point_cloud->height = 1;
            point_cloud->header.stamp = static_cast<uint64_t>(sweep_end_time * 1e6);
            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_scan(new pcl::PointCloud <pcl::PointXYZI>);
            point_cloud_scan->header.frame_id = frame_id;
            point_cloud_scan->height = 1;
            point_cloud_scan->header.stamp = static_cast<uint64_t>(sweep_end_time * 1e6);
            size_t j;
            pcl::PointXYZI point;
            pcl::PointXYZI scan_point;
            if (!sweep_data_bak->points.empty()) {
                for (j = 0; j < sweep_data_bak->points.size(); ++j) {
                    if ((sweep_data_bak->points[j].azimuth > angle_disable_min) &&
                        (sweep_data_bak->points[j].azimuth < angle_disable_max)) {
                        continue;
                    }
                    if (scan_num == sweep_data_bak->points[j].ring) {
                        scan_point.x = sweep_data_bak->points[j].x;
                        scan_point.y = sweep_data_bak->points[j].y;
                        scan_point.z = sweep_data_bak->points[j].z;
                        scan_point.intensity = sweep_data_bak->points[j].intensity;
                        point_cloud_scan->points.push_back(scan_point);
                        ++point_cloud_scan->width;
                    }
                    point.x = sweep_data_bak->points[j].x;
                    point.y = sweep_data_bak->points[j].y;
                    point.z = sweep_data_bak->points[j].z;
                    point.intensity = sweep_data_bak->points[j].intensity;
                    point_cloud->points.push_back(point);
                    ++point_cloud->width;
                }
            }
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            pointcloud_pub->publish(pc_msg);
            if (publish_scan) {
                sensor_msgs::msg::PointCloud2 pc_msg2;
                sensor_msgs::msg::LaserScan::UniquePtr scan_msg(new sensor_msgs::msg::LaserScan);
                pcl::toROSMsg(*point_cloud_scan, pc_msg2);
                pointcloudToLaserscan(pc_msg2, *scan_msg);
                scan_pub->publish(std::move(scan_msg));
            }

        } else {
            VPointcloud::Ptr point_cloud(new VPointcloud());
            point_cloud->header.frame_id = frame_id;
            point_cloud->height = 1;
            point_cloud->header.stamp = static_cast<uint64_t>(sweep_end_time * 1e6);
            VPointcloud::Ptr point_cloud_scan(new VPointcloud());
            point_cloud_scan->header.frame_id = frame_id;
            point_cloud_scan->height = 1;
            point_cloud_scan->header.stamp = static_cast<uint64_t>(sweep_end_time * 1e6);
            size_t j;
            VPoint point;
            VPoint scan_point;
            if (!sweep_data_bak->points.empty()) {
                for (j = 0; j < sweep_data_bak->points.size(); ++j) {
                    if ((sweep_data_bak->points[j].azimuth > angle_disable_min) &&
                        (sweep_data_bak->points[j].azimuth < angle_disable_max)) {
                        continue;
                    }
                    if (scan_num == sweep_data_bak->points[j].ring) {
                        scan_point.x = sweep_data_bak->points[j].x;
                        scan_point.y = sweep_data_bak->points[j].y;
                        scan_point.z = sweep_data_bak->points[j].z;
                        scan_point.intensity = sweep_data_bak->points[j].intensity;
                        point_cloud_scan->points.push_back(scan_point);
                        ++point_cloud_scan->width;
                    }

                    point.x = sweep_data_bak->points[j].x;
                    point.y = sweep_data_bak->points[j].y;
                    point.z = sweep_data_bak->points[j].z;
                    point.intensity = sweep_data_bak->points[j].intensity;
                    point.ring = sweep_data_bak->points[j].ring;
                    point.time = sweep_data_bak->points[j].time;
                    point_cloud->points.push_back(point);
                    ++point_cloud->width;
                    current_point_time = point.time;
                    if (current_point_time - last_point_time < 0.0) {
                        //ROS_WARN("timestamp is rolled back! current point time: %.12f  last point time: %.12f", current_point_time, last_point_time);
                    }
                    last_point_time = current_point_time;
                }
            }
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            pointcloud_pub->publish(pc_msg);

            if (publish_scan) {
                sensor_msgs::msg::PointCloud2 pc_msg2;
                sensor_msgs::msg::LaserScan::UniquePtr scan_msg(new sensor_msgs::msg::LaserScan());
                pcl::toROSMsg(*point_cloud_scan, pc_msg2);
                pointcloudToLaserscan(pc_msg2, *scan_msg);
                scan_pub->publish(std::move(scan_msg));
            }
        }
        return;
    }

    void LslidarDriver::publishScan() {
        sensor_msgs::msg::LaserScan::UniquePtr scan(new sensor_msgs::msg::LaserScan);
        int layer_num_local = scan_num;
        RCLCPP_INFO(this->get_logger(), "default channel is %d", layer_num_local);

        scan->header.frame_id = frame_id;
        scan->header.stamp = rclcpp::Time(sweep_end_time);

        scan->angle_min = 0.0;
        scan->angle_max = 2.0 * M_PI;
        scan->angle_increment = (scan->angle_max - scan->angle_min) / point_num;

        //	scan->time_increment = motor_speed_/1e8;
        scan->range_min = min_range;
        scan->range_max = max_range;
        scan->ranges.reserve(point_num);
        scan->ranges.assign(point_num, std::numeric_limits<float>::quiet_NaN());

        scan->intensities.reserve(point_num);
        scan->intensities.assign(point_num, std::numeric_limits<float>::quiet_NaN());

        if (sweep_data->points.size() > 0) {
            for (size_t j = 0; j < sweep_data->points.size(); ++j) {
                if (layer_num_local == sweep_data->points[j].ring) {
                    float point_azimuth = sweep_data->points[j].azimuth * 0.01 * DEG_TO_RAD;
                    uint point_idx = point_azimuth / scan->angle_increment;
                    point_idx = point_idx <= point_num ? point_idx : point_idx % point_num;
                    RCLCPP_INFO(this->get_logger(), "point_idx: %d", point_idx);
                    scan->ranges[point_idx] = sweep_data->points[j].distance;
                    scan->intensities[point_idx] = sweep_data->points[j].intensity;
                }
            }
            scan_pub->publish(std::move(scan));
        }
    }

    void LslidarDriver::decodePacket(const RawPacket *packet) {
        //couputer azimuth angle for each firing
        for (size_t b_idx = 0; b_idx < BLOCKS_PER_PACKET; ++b_idx) {
            firings.firing_azimuth[b_idx] = packet->blocks[b_idx].rotation % 36000; //* 0.01 * DEG_TO_RAD;
        }
        for (size_t block_idx = 0; block_idx < BLOCKS_PER_PACKET; ++block_idx) {
            const RawBlock &raw_block = packet->blocks[block_idx];
            // computer distance ,intensity
            //      for (size_t blk_fir_idx = 0; blk_fir_idx < FIRINGS_PER_BLOCK; ++blk_fir_idx) {
            //        size_t fir_idx = blk_idx * FIRINGS_PER_BLOCK + blk_fir_idx;
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
                firings.azimuth[block_idx * 32 + scan_fir_idx] = firings.firing_azimuth[block_idx] +
                                                                 scan_fir_idx * azimuth_diff_b / FIRING_TOFFSET_C8;
                firings.azimuth[block_idx * 32 + scan_fir_idx] = firings.azimuth[block_idx * 32 + scan_fir_idx] % 36000;
                // distance
                TwoBytes raw_distance{};
                raw_distance.bytes[0] = raw_block.data[byte_idx];
                raw_distance.bytes[1] = raw_block.data[byte_idx + 1];
                firings.distance[block_idx * 32 + scan_fir_idx] =
                        static_cast<double>(raw_distance.distance) * DISTANCE_RESOLUTION * distance_unit;

                //intensity
                firings.intensity[block_idx * 32 + scan_fir_idx] = static_cast<double>(
                        raw_block.data[byte_idx + 2]);
            }
        }

        return;
    }

    bool LslidarDriver::poll() {
        //ROS_INFO("lidar_number: %d",lidar_number_);
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet(new lslidar_msgs::msg::LslidarPacket());
        // Since the rslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            int rc = msop_input_->getPacket(packet);
            if (rc == 0) break;
            if (rc < 0) return false;
        }

        // packet timestamp
        if (use_gps_ts) {
            lslidar_msgs::msg::LslidarPacket pkt = *packet;
            if (time_service_mode_ == 1) {    //ptp授时
                //std::cout << "ptp";
                uint64_t timestamp_s = (pkt.data[1201] * pow(2, 32) + pkt.data[1202] * pow(2, 24) +
                                        pkt.data[1203] * pow(2, 16) +
                                        pkt.data[1204] * pow(2, 8) + pkt.data[1205] * pow(2, 0));
                uint64_t timestamp_nsce = pkt.data[1206] +
                                          pkt.data[1207] * pow(2, 8) +
                                          pkt.data[1208] * pow(2, 16) +
                                          pkt.data[1209] * pow(2, 24); //ns
                timeStamp = rclcpp::Time(timestamp_s, timestamp_nsce);// s,ns
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
            } else if (time_service_mode_ == 0){          //gps授时
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_year = pkt.data[1200] + 2000 - 1900;
                cur_time.tm_mon = pkt.data[1201] - 1;
                cur_time.tm_mday = pkt.data[1202];
                cur_time.tm_hour = pkt.data[1203];
                cur_time.tm_min = pkt.data[1204];
                cur_time.tm_sec = pkt.data[1205];
                packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = pkt.data[1206] +
                                 pkt.data[1207] * pow(2, 8) +
                                 pkt.data[1208] * pow(2, 16) +
                                 pkt.data[1209] * pow(2, 24); //ns
                timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
            }else if (time_service_mode_ == 2){          //ntp授时
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_year = pkt.data[1200] + 2000 - 1900;
                cur_time.tm_mon = pkt.data[1201] - 1;
                cur_time.tm_mday = pkt.data[1202];
                cur_time.tm_hour = pkt.data[1203];
                cur_time.tm_min = pkt.data[1204];
                cur_time.tm_sec = pkt.data[1205];
                packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = (pkt.data[1206] +
                                 pkt.data[1207] * pow(2, 8) +
                                 pkt.data[1208] * pow(2, 16) +
                                 pkt.data[1209] * pow(2, 24)) * 1e3; //ns
                timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
            }
        } else {
            packet->stamp = this->get_clock()->now();
            current_packet_time = this->get_clock()->now().seconds();
        }
        if (packet->data[1210] == 0x39) {
            return_mode = 2;
        }
        RCLCPP_INFO_ONCE(this->get_logger(), "return mode: %d", return_mode);
        const RawPacket *raw_packet = (const RawPacket *) (&(packet->data[0]));
        //check if the packet is valid
        if (!checkPacketValidity(raw_packet)) return false;

        //decode the packet
        decodePacket(raw_packet);
        // find the start of a new revolution
        // if there is one, new_sweep_start will be the index of the start firing,
        // otherwise, new_sweep_start will be FIRINGS_PER_PACKET.
        size_t new_sweep_start = 0;
        do {
            if (abs(firings.azimuth[new_sweep_start] - last_azimuth) > 35000) {
                break;
            } else {
                last_azimuth = firings.azimuth[new_sweep_start];
                ++new_sweep_start;
            }
        } while (new_sweep_start < SCANS_PER_PACKET);
        // The first sweep may not be complete. So, the firings with
        // the first sweep will be discarded. We will wait for the
        // second sweep in order to find the 0 azimuth angle.
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
        for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
            if ("c32_70" == c32_type) {
                if (fir_idx % 32 == 29 || fir_idx % 32 == 6 || fir_idx % 32 == 14 || fir_idx % 32 == 22 ||
                    fir_idx % 32 == 30 || fir_idx % 32 == 7 || fir_idx % 32 == 15 || fir_idx % 32 == 23) {
//                    ROS_INFO("firings.azimuth[fir_idx] +=389;");
                    firings.azimuth[fir_idx] += 389;
                }
                if (firings.azimuth[fir_idx] > 36000) firings.azimuth[fir_idx] -= 36000;
            }
            //check if the point is valid
            if (!isPointInRange(firings.distance[fir_idx]))continue;
            //convert the point to xyz coordinate
            size_t table_idx = firings.azimuth[fir_idx];
            double cos_azimuth = cos_azimuth_table[table_idx];
            double sin_azimuth = sin_azimuth_table[table_idx];
            double x_coord, y_coord, z_coord;
            double R1 = (c32_type == "c32_90") ? R1_90 : R1_;
            double conversionAngle = (c32_type == "c32_90") ? conversionAngle_90 : conversionAngle_;
            if (coordinate_opt) {
                x_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * cos_azimuth +
                          R1 * cos((conversionAngle - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                y_coord = -firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * sin_azimuth +
                          R1 * sin((conversionAngle - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                z_coord = firings.distance[fir_idx] * sin_scan_altitude[fir_idx % lidar_number_];

            } else {
                //Y-axis correspondence 0 degree
                x_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * sin_azimuth +
                          R1 * sin((firings.azimuth[fir_idx] * 0.01 - conversionAngle) * DEG_TO_RAD);
                y_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * cos_azimuth +
                          R1 * cos((firings.azimuth[fir_idx] * 0.01 - conversionAngle) * DEG_TO_RAD);
                z_coord = firings.distance[fir_idx] * sin_scan_altitude[fir_idx % lidar_number_];

            }
            // computer the time of the point
            double time;
            if (last_packet_time > 1e-6) {
                time =
                        current_packet_time -
                        (current_packet_time - last_packet_time) * (SCANS_PER_PACKET - fir_idx - 1) / SCANS_PER_PACKET;
            } else {
                time = current_packet_time;
            }
            int remapped_scan_idx = 0;
            switch (lidar_number_) {
                case 1:
                    remapped_scan_idx = 0;
                    break;
                case 8:
                    remapped_scan_idx = (fir_idx % 8) % 2 * 4 + (fir_idx % 8) / 2;
                    break;
                case 16:
                    remapped_scan_idx = (fir_idx % 16) % 2 == 0 ? (fir_idx % 16) / 2 : (fir_idx % 16) / 2 + 8;
                    break;
                case 32:
                    remapped_scan_idx = (fir_idx % 32) % 4 * 8 + fir_idx % 32 / 4;
                    break;
                default:
                    remapped_scan_idx = 0;
                    break;
            }

            sweep_data->points.push_back(lslidar_msgs::msg::LslidarPoint());
            lslidar_msgs::msg::LslidarPoint &new_point = sweep_data->points[
                    sweep_data->points.size() - 1];
            //pack the data into point msg
            new_point.time = time;
            new_point.x = x_coord;
            new_point.y = y_coord;
            new_point.z = z_coord;
            new_point.intensity = firings.intensity[fir_idx];
            new_point.ring = remapped_scan_idx;
            new_point.azimuth = firings.azimuth[fir_idx];
            new_point.distance = firings.distance[fir_idx];
        }
        // a new sweep begins ----------------------------------------------------
        if (end_fir_idx != SCANS_PER_PACKET) {
            //publish Last frame scan
            if (last_packet_time > 1e-6) {
                sweep_end_time =
                        current_packet_time -
                        (current_packet_time - last_packet_time) * (SCANS_PER_PACKET - end_fir_idx) / SCANS_PER_PACKET;
            } else {
                sweep_end_time = current_packet_time;
            }
            sweep_end_time = sweep_end_time > 0 ? sweep_end_time : 0;
            {
                std::unique_lock <std::mutex> lock(pointcloud_lock);
                sweep_data_bak = std::move(sweep_data);
            }
            std::thread pointcloud_pub_thread([this] { publishPointcloud(); });
            pointcloud_pub_thread.detach();

//            if (publish_scan) publishScan();
            sweep_data = std::make_unique<lslidar_msgs::msg::LslidarScan>();

            //prepare the next frame scan
            //sweep_end_time = packet->stamp.toSec() - (end_fir_idx - start_fir_idx) * 3.125 * 1e-6;
            last_azimuth = firings.azimuth[SCANS_PER_PACKET - 1];
            start_fir_idx = end_fir_idx;
            end_fir_idx = SCANS_PER_PACKET;
            for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
                if ("c32_70" == c32_type) {
                    if (fir_idx % 32 == 29 || fir_idx % 32 == 6 || fir_idx % 32 == 14 || fir_idx % 32 == 22 ||
                        fir_idx % 32 == 30 || fir_idx % 32 == 7 || fir_idx % 32 == 15 || fir_idx % 32 == 23) {
//                    ROS_INFO("firings.azimuth[fir_idx] +=389;");
                        firings.azimuth[fir_idx] += 389;
                    }
                    if (firings.azimuth[fir_idx] > 36000) firings.azimuth[fir_idx] -= 36000;
                }

                //check if the point is valid
                if (!isPointInRange(firings.distance[fir_idx])) continue;
                //convert the point to xyz coordinate
                size_t table_idx = firings.azimuth[fir_idx];
                double cos_azimuth = cos_azimuth_table[table_idx];
                double sin_azimuth = sin_azimuth_table[table_idx];
                double x_coord, y_coord, z_coord;
                double R1 = (c32_type == "c32_90") ? R1_90 : R1_;
                double conversionAngle = (c32_type == "c32_90") ? conversionAngle_90 : conversionAngle_;
                if (coordinate_opt) {
                    x_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * cos_azimuth +
                              R1 * cos((conversionAngle - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                    y_coord = -firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * sin_azimuth +
                              R1 * sin((conversionAngle - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                    z_coord = firings.distance[fir_idx] * sin_scan_altitude[fir_idx % lidar_number_];

                } else {
                    //Y-axis correspondence 0 degree
                    x_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * sin_azimuth +
                              R1 * sin((firings.azimuth[fir_idx] * 0.01 - conversionAngle) * DEG_TO_RAD);
                    y_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * cos_azimuth +
                              R1 * cos((firings.azimuth[fir_idx] * 0.01 - conversionAngle) * DEG_TO_RAD);
                    z_coord = firings.distance[fir_idx] * sin_scan_altitude[fir_idx % lidar_number_];

                }
                // computer the time of the point
                double time;
                if (last_packet_time > 1e-6) {
                    time =
                            current_packet_time -
                            (current_packet_time - last_packet_time) * (SCANS_PER_PACKET - fir_idx - 1) /
                            SCANS_PER_PACKET;
                } else {
                    time = current_packet_time;
                }

                int remapped_scan_idx = 0;
                switch (lidar_number_) {
                    case 1:
                        remapped_scan_idx = 0;
                        break;
                    case 8:
                        remapped_scan_idx = (fir_idx % 8) % 2 * 4 + (fir_idx % 8) / 2;
                        break;
                    case 16:
                        remapped_scan_idx = (fir_idx % 16) % 2 == 0 ? (fir_idx % 16) / 2 : (fir_idx % 16) / 2 + 8;
                        break;
                    case 32:
                        remapped_scan_idx = (fir_idx % 32) % 4 * 8 + fir_idx % 32 / 4;
                        break;
                    default:
                        remapped_scan_idx = 0;
                        break;
                }

                sweep_data->points.push_back(lslidar_msgs::msg::LslidarPoint());
                lslidar_msgs::msg::LslidarPoint &new_point = sweep_data->points[
                        sweep_data->points.size() - 1];
                //pack the data into point msg
                new_point.time = time;
                new_point.x = x_coord;
                new_point.y = y_coord;
                new_point.z = z_coord;
                new_point.intensity = firings.intensity[fir_idx];
                new_point.ring = remapped_scan_idx;
                new_point.azimuth = firings.azimuth[fir_idx];
                new_point.distance = firings.distance[fir_idx];
            }
        }
        last_packet_time = current_packet_time;
        //packet_pub.publish(*packet);
        return true;
    }

    void LslidarDriver::setPacketHeader(unsigned char *config_data) {
        config_data[0] = 0xAA;
        config_data[1] = 0x00;
        config_data[2] = 0xFF;
        config_data[3] = 0x11;
        config_data[4] = 0x22;
        config_data[5] = 0x22;
        config_data[6] = 0xAA;
        config_data[7] = 0xAA;
    }

    bool LslidarDriver::sendPacketTolidar(unsigned char *config_data) const {
        int socketid;
        sockaddr_in addrSrv{};
        socketid = socket(2, 2, 0);
        addrSrv.sin_addr.s_addr = inet_addr(lidar_ip_string.c_str());
        addrSrv.sin_family = AF_INET;
        addrSrv.sin_port = htons(2368);
        sendto(socketid, (const char *) config_data, 1206, 0, (struct sockaddr *) &addrSrv, sizeof(addrSrv));
        return true;
    }

    bool LslidarDriver::powerOn(std::shared_ptr <lslidar_msgs::srv::LslidarControl::Request> req,
                                std::shared_ptr <lslidar_msgs::srv::LslidarControl::Response> res) {
        RCLCPP_INFO(this->get_logger(), "--------------------------");
        // sleep(1);
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet0(new lslidar_msgs::msg::LslidarPacket);
        packet0->data[0] = 0x00;
        packet0->data[1] = 0x00;
        int rc_msop = -1;

        if(!is_get_difop_) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        if (req->laser_control == 1) {
            if ((rc_msop = msop_input_->getPacket(packet0)) == 0) {
                res->result = 1;
                RCLCPP_WARN(this->get_logger(), "receive cmd: %d,already power on status", req->laser_control);
                return true;
            }
            RCLCPP_WARN(this->get_logger(), "receive cmd: %d,power on", req->laser_control);
            config_data[50] = 0xBB;
            sendPacketTolidar(config_data);
            double time1 = this->get_clock()->now().seconds();

            do {
                rc_msop = msop_input_->getPacket(packet0);
                double time2 = this->get_clock()->now().seconds();
                if (time2 - time1 > 20) {
                    res->result = 0;
                    RCLCPP_ERROR(this->get_logger(), "lidar connect error");
                    return true;
                }
            } while ((rc_msop != 0) && (packet0->data[0] != 0xff) && (packet0->data[1] != 0xee));
            sleep(5);
            res->result = 1;
        } else if (req->laser_control == 0) {
            config_data[50] = 0xAA;
            RCLCPP_INFO(this->get_logger(), "receive cmd: %d,power off", req->laser_control);
            sendPacketTolidar(config_data);
            res->result = 1;
        } else {
            RCLCPP_ERROR(this->get_logger(), "cmd error");
            res->result = 0;
        }
        return true;
    }

    bool LslidarDriver::timeService(std::shared_ptr <lslidar_msgs::srv::TimeService::Request> req,
                                    std::shared_ptr <lslidar_msgs::srv::TimeService::Response> res) {
        RCLCPP_INFO(this->get_logger(), "Start to modify lidar time service mode");
        if(!is_get_difop_) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        std::string time_service_mode = req->time_service_mode;
        transform(time_service_mode.begin(), time_service_mode.end(), time_service_mode.begin(), ::tolower);

        if (time_service_mode == "gps") {
            config_data[45] = 0x00;
        } else if (time_service_mode == "ptp") {
            config_data[45] = 0x01;
        } else if (time_service_mode == "ntp") {
            config_data[45] = 0x02;
            std::string ntp_ip = req->ntp_ip;
            std::regex ipv4(
                    "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
            if (!regex_match(ntp_ip, ipv4)) {
                RCLCPP_ERROR(this->get_logger(),"Parameter error, please check the input parameters");
                res->result = false;
                return true;
            }
            unsigned short first_value, second_value, third_value, end_value;
            sscanf(ntp_ip.c_str(), "%hu.%hu.%hu.%hu", &first_value, &second_value, &third_value, &end_value);
            config_data[28] = first_value;
            config_data[29] = second_value;
            config_data[30] = third_value;
            config_data[31] = end_value;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Parameter error, please check the input parameters");
            res->result = false;
            return true;
        }
        printf("byte[45] %X \n", config_data[45]);
        res->result = true;
        sendPacketTolidar(config_data);
        RCLCPP_INFO(this->get_logger(), "Time service method modified successfully!");
        return true;
    }

    bool LslidarDriver::motorControl(std::shared_ptr<lslidar_msgs::srv::MotorControl::Request> req,
                                     std::shared_ptr<lslidar_msgs::srv::MotorControl::Response> res) {
        if(!is_get_difop_) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        if (req->motor_control == 1) {
            config_data[41] = 0x00;
        } else if (req->motor_control == 0) {
            config_data[41] = 0x01;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Parameter error, please check the input parameters");
            res->result = false;
            return true;
        }
        printf("byte[41] %X \n", config_data[41]);
        res->result = true;
        sendPacketTolidar(config_data);
        RCLCPP_INFO(this->get_logger(), "Set successfully!");
        return true;
    }

    bool LslidarDriver::motorSpeed(std::shared_ptr <lslidar_msgs::srv::MotorSpeed::Request> req,
                                   std::shared_ptr <lslidar_msgs::srv::MotorSpeed::Response> res) {
        if(!is_get_difop_) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        if (req->motor_speed == 5) {
            config_data[8] = 0x01;
            config_data[9] = 0x2c;
        } else if (req->motor_speed == 10) {
            config_data[8] = 0x02;
            config_data[9] = 0x58;
        } else if (req->motor_speed == 20) {
            config_data[8] = 0x04;
            config_data[9] = 0xB0;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Parameter error, please check the input parameters");
            res->result = false;
            return true;
        }
        res->result = true;
        sendPacketTolidar(config_data);
        RCLCPP_INFO(this->get_logger(), "Set successfully!");
        return true;
    }

    bool LslidarDriver::setDataPort(std::shared_ptr <lslidar_msgs::srv::DataPort::Request> req,
                                    std::shared_ptr <lslidar_msgs::srv::DataPort::Response> res) {
        if(!is_get_difop_) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        int dev_port = config_data[26] * 256 + config_data[27];
        if (req->data_port < 1025 || req->data_port > 65535 || req->data_port == dev_port) {
            RCLCPP_ERROR(this->get_logger(), "Parameter error, please check the input parameters");
            res->result = false;
            return true;
        } else {
            config_data[24] = req->data_port / 256;
            config_data[25] = req->data_port % 256;
        }
        res->result = true;
        sendPacketTolidar(config_data);
        RCLCPP_INFO(this->get_logger(), "Set successfully!");
        return true;
    }

    bool LslidarDriver::setDevPort(std::shared_ptr <lslidar_msgs::srv::DevPort::Request> req,
                                   std::shared_ptr <lslidar_msgs::srv::DevPort::Response> res) {
        if(!is_get_difop_) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        int data_port = config_data[24] * 256 + config_data[25];
        if (req->dev_port < 1025 || req->dev_port > 65535 || req->dev_port == data_port) {
            RCLCPP_ERROR(this->get_logger(), "Parameter error, please check the input parameters");
            res->result = false;
            return true;
        } else {
            config_data[26] = req->dev_port / 256;
            config_data[27] = req->dev_port % 256;
        }
        res->result = true;
        sendPacketTolidar(config_data);
        RCLCPP_INFO(this->get_logger(), "Set successfully!");
        return true;
    }

    bool LslidarDriver::setDataIp(std::shared_ptr <lslidar_msgs::srv::DataIp::Request> req,
                                  std::shared_ptr <lslidar_msgs::srv::DataIp::Response> res) {
        std::regex ipv4(
                "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
        if (!regex_match(req->data_ip, ipv4)) {
            RCLCPP_ERROR(this->get_logger(), "Parameter error, please check the input parameters");
            res->result = false;
            return true;
        }

        if(!is_get_difop_) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        unsigned short first_value, second_value, third_value, end_value;
        sscanf(req->data_ip.c_str(), "%hu.%hu.%hu.%hu", &first_value, &second_value, &third_value, &end_value);

        std::string destination_ip = std::to_string(config_data[14]) + "." + std::to_string(config_data[15]) + "." +
                                     std::to_string(config_data[16]) + "." + std::to_string(config_data[17]);
        if (first_value == 0 || first_value == 127 ||
            (first_value >= 240 && first_value <= 255) || destination_ip == req->data_ip) {
            RCLCPP_ERROR(this->get_logger(), "Parameter error, please check the input parameters");
            res->result = false;
            return true;
        } else {
            config_data[10] = first_value;
            config_data[11] = second_value;
            config_data[12] = third_value;
            config_data[13] = end_value;
        }
        res->result = true;
        sendPacketTolidar(config_data);
        RCLCPP_INFO(this->get_logger(), "Set successfully!");
        return true;
    }

    bool LslidarDriver::setDestinationIp(std::shared_ptr <lslidar_msgs::srv::DestinationIp::Request> req,
                                         std::shared_ptr <lslidar_msgs::srv::DestinationIp::Response> res) {
        std::regex ipv4(
                "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
        if (!regex_match(req->destination_ip, ipv4)) {
            RCLCPP_ERROR(this->get_logger(), "Parameter error, please check the input parameters");
            res->result = false;
            return true;
        }

        if(!is_get_difop_) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            RCLCPP_ERROR(this->get_logger(), "Can not get dev packet! Set failed!");
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        unsigned short first_value, second_value, third_value, end_value;
        sscanf(req->destination_ip.c_str(), "%hu.%hu.%hu.%hu", &first_value, &second_value, &third_value, &end_value);

        std::string data_ip = std::to_string(config_data[10]) + "." + std::to_string(config_data[11]) + "." +
                              std::to_string(config_data[12]) + "." + std::to_string(config_data[13]);
        if (first_value == 0 || first_value == 127 ||
            (first_value >= 240 && first_value <= 255) || data_ip == req->destination_ip) {
            RCLCPP_ERROR(this->get_logger(), "Parameter error, please check the input parameters");
            res->result = false;
            return true;
        } else {
            config_data[14] = first_value;
            config_data[15] = second_value;
            config_data[16] = third_value;
            config_data[17] = end_value;
        }
        res->result = true;
        sendPacketTolidar(config_data);
        RCLCPP_INFO(this->get_logger(), "Set successfully!");
        return true;
    }

    void LslidarDriver::pollThread(void) {
        while (rclcpp::ok()) {
            poll();
        }
    }

}  // namespace lslidar_driver

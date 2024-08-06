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

    LslidarDriver::LslidarDriver(const rclcpp::NodeOptions &options) : Node("lslidar_node", options),
                                                                       last_azimuth(0),
                                                                       is_first_sweep(true),
                                                                       sweep_end_time(0.0),
                                                                       return_mode(1),
                                                                       lidar_number_(1),
                                                                       point_num(2000),
                                                                       packet_rate(1695.0),
                                                                       current_packet_time(0.0),
                                                                       last_packet_time(0.0),
                                                                       horizontal_angle_resolution(0.0), 
                                                                       is_get_difop_(false),
                                                                       start_process_msop_(false),
                                                                       point_cloud_xyzi_(new pcl::PointCloud<pcl::PointXYZI>()),
                                                                       point_cloud_xyzi_bak_(new pcl::PointCloud<pcl::PointXYZI>()),
                                                                       point_cloud_xyzirt_(new pcl::PointCloud<VPoint>()),
                                                                       point_cloud_xyzirt_bak_(new pcl::PointCloud<VPoint>()),
                                                                       scan_msg(new sensor_msgs::msg::LaserScan()),
                                                                       scan_msg_bak(new sensor_msgs::msg::LaserScan()){
                                                                    
        return;
    }

    bool LslidarDriver::checkPacketValidity(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) {
        for (size_t blk_idx = 0; blk_idx < 6; ++blk_idx) {
            if (packet->data[blk_idx * 100] != 0xff && packet->data[blk_idx * 100 + 1] != 0xee) {
                return false;
            }
        }
        return true;
    }

    // bool LslidarDriver::isPointInRange(const double &distance) {
    //     return (distance >= min_range && distance < max_range);
    // }

    bool LslidarDriver::loadParameters() {
        this->declare_parameter<std::string>("pcap", "");
        this->declare_parameter<double>("packet_rate", 1695.0);
        this->declare_parameter<std::string>("device_ip", "192.168.1.200");
        this->declare_parameter<int>("msop_port", 2368);
        this->declare_parameter<int>("difop_port", 2369);
        this->declare_parameter<bool>("pcl_type", false);
        this->declare_parameter<bool>("add_multicast", false);
        this->declare_parameter<std::string>("group_ip", "234.2.3.2");
        this->declare_parameter<bool>("use_time_service", false);
        this->declare_parameter<double>("min_range", 0.3);
        this->declare_parameter<double>("max_range", 150.0);
        this->declare_parameter<int>("angle_disable_min", 0);
        this->declare_parameter<int>("angle_disable_max", 0);
        this->declare_parameter<double>("distance_unit", 0.4);
        this->declare_parameter<double>("horizontal_angle_resolution", 0.18);
        this->declare_parameter<std::string>("frame_id", "laser_link");
        this->declare_parameter<std::string>("topic_name", "lslidar_point_cloud");
        this->declare_parameter<bool>("publish_scan", false);
        this->declare_parameter<int>("scan_num", 15);
        this->declare_parameter<bool>("coordinate_opt", false);

        this->get_parameter("pcap", dump_file);
        this->get_parameter("packet_rate", packet_rate);
        this->get_parameter("device_ip", lidar_ip_string);
        this->get_parameter("msop_port", msop_udp_port);
        this->get_parameter("difop_port", difop_udp_port);
        this->get_parameter("pcl_type", pcl_type);
        this->get_parameter("add_multicast", add_multicast);
        this->get_parameter("group_ip", group_ip_string);
        this->get_parameter("use_time_service", use_time_service);
        this->get_parameter("min_range", min_range);
        this->get_parameter("max_range", max_range);
        this->get_parameter("angle_disable_min", angle_disable_min);
        this->get_parameter("angle_disable_max", angle_disable_max);
        this->get_parameter("distance_unit", distance_unit);
        this->get_parameter("horizontal_angle_resolution", horizontal_angle_resolution);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("topic_name", pointcloud_topic);
        this->get_parameter("publish_scan", publish_scan);
        this->get_parameter("scan_num", scan_num);
        this->get_parameter("coordinate_opt", coordinate_opt);

        return true;
    }

    void LslidarDriver::outputParameters() {
        LS_PARAM << "******** CX ROS2 driver version: 4.2.4 ********" << LS_END;
        LS_PARAM << "dump_file: " << dump_file.c_str() << LS_END;
        LS_PARAM << "packet_rate: " << packet_rate << LS_END;
        LS_PARAM << "pcl_type: " << pcl_type << LS_END;
        LS_PARAM << "add_multicast: " << add_multicast << LS_END;
        LS_PARAM << "use_time_service: " << use_time_service << LS_END;
        LS_PARAM << "min_range: " << min_range << LS_END;
        LS_PARAM << "max_range: "<< max_range << LS_END;
        LS_PARAM << "angle_disable_min: " << angle_disable_min << LS_END;
        LS_PARAM << "angle_disable_max: " << angle_disable_max << LS_END;
        LS_PARAM << "horizontal_angle_resolution: " << horizontal_angle_resolution << LS_END;
        LS_PARAM << "frame_id: " << frame_id.c_str() << LS_END;
        LS_PARAM << "pointcloud_topic: " << pointcloud_topic.c_str() << LS_END;
        LS_PARAM << "publish_scan: " << publish_scan << LS_END;
        LS_PARAM << "coordinate_opt: " << coordinate_opt << LS_END;
        if (publish_scan) LS_INFO << "scan_num: " << scan_num << LS_END;

        LS_SOCKET << "Accepting packets from IP address: " << lidar_ip_string.c_str() << LS_END;
        if (add_multicast) LS_INFO << "Adding multicast groupaddress: " << group_ip_string.c_str() << LS_END;
        LS_SOCKET << "Opening UDP socket msop port: " << msop_udp_port << LS_END;
        LS_SOCKET << "Opening UDP socket difop port: " << difop_udp_port << LS_END;
        std::cout << std::endl;
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
            LS_ERROR << "cannot load all required ROS parameters." << LS_END;
            return false;
        }

        outputParameters();

        if (!createRosIO()) {
            LS_ERROR << "cannot create all ROS IO." << LS_END;
            return false;
        }

        while (angle_disable_min < 0)
			angle_disable_min += 36000;
		while (angle_disable_max < 0)
			angle_disable_max += 36000;
		while (angle_disable_min > 36000)
			angle_disable_min -= 36000;
		while (angle_disable_max > 36000)
			angle_disable_max -= 36000;
		if (angle_disable_max == angle_disable_min)
		{
			angle_able_min = 0;
			angle_able_max = 36000;
		}
		else
		{
			if (angle_disable_min < angle_disable_max && angle_disable_min != 0)
			{
				angle_able_min = angle_disable_max;
				angle_able_max = angle_disable_min + 36000;
			}
			if (angle_disable_min < angle_disable_max && angle_disable_min == 0)
			{
				angle_able_min = angle_disable_max;
				angle_able_max = 36000;
			}
			if (angle_disable_min > angle_disable_max)
			{
				angle_able_min = angle_disable_max;
				angle_able_max = angle_disable_min;
			}
		}
  
        // create the sin and cos table for different azimuth values
        for (int j = 0; j < 36000; ++j) {
            double angle = static_cast<double>(j) / 100.0 * DEG_TO_RAD;
            sin_azimuth_table[j] = sin(angle);
            cos_azimuth_table[j] = cos(angle);
        }

        point_cloud_xyzirt_->header.frame_id = frame_id;
        point_cloud_xyzirt_->height = 1;

        point_cloud_xyzi_->header.frame_id = frame_id;
        point_cloud_xyzi_->height = 1;
        if (publish_scan) {
            scan_msg->angle_min = 0;
            scan_msg->angle_max = M_PI * 2;
            scan_msg->range_min = min_range;
            scan_msg->range_max = max_range;
            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
            point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        }

        return true;
    }

    void LslidarDriver::difopPoll() {
        // reading and publishing scans as fast as possible.
        lslidar_msgs::msg::LslidarPacket::UniquePtr difop_packet_ptr(new lslidar_msgs::msg::LslidarPacket());
        static bool is_print_working_time = true;
        while (rclcpp::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet_ptr);
            
            if (rc == 0) {
                if (difop_packet_ptr->data[0] != 0xa5 || difop_packet_ptr->data[1] != 0xff ||
                    difop_packet_ptr->data[2] != 0x00 || difop_packet_ptr->data[3] != 0x5a) {
                    return;
                }

                for (int i = 0; i < 1206; i++) {
                    difop_data[i] = difop_packet_ptr->data[i];
                }

                if (difop_packet_ptr->data[1196] == 0x03 || (difop_packet_ptr->data[1196] == 0x02 && difop_packet_ptr->data[1197] / 16 == 8)) {
                    fpga_type = 3;
                    for (int i = 0; i < 32; ++i) {
                        uint8_t data1 = difop_packet_ptr->data[245 + 2 * i];
                        uint8_t data2 = difop_packet_ptr->data[245 + 2 * i + 1];
                        int vert_angle = data1 * 256 + data2;
                        vert_angle = vert_angle > 32767 ? (vert_angle - 65535) : vert_angle;
                        config_vertical_angle_32[i] = vert_angle * 0.01;
                    }

                    int angle_a0 = difop_packet_ptr->data[186] * 256 + difop_packet_ptr->data[187];
                    adjust_angle[0] = angle_a0 > 32767 ? 32767 - angle_a0 : angle_a0;

                    int angle_a1 = difop_packet_ptr->data[190] * 256 + difop_packet_ptr->data[191];
                    adjust_angle[1] = angle_a1 > 32767 ? 32767 - angle_a1 : angle_a1;

                    int angle_a2 = difop_packet_ptr->data[188] * 256 + difop_packet_ptr->data[189];
                    adjust_angle[2] = angle_a2 > 32767 ? 32767 - angle_a2 : angle_a2;

                    int angle_a3 = difop_packet_ptr->data[192] * 256 + difop_packet_ptr->data[193];
                    adjust_angle[3] = angle_a3 > 32767 ? 32767 - angle_a3 : angle_a3;

                    this->packetTimeStamp[4] = difop_packet_ptr->data[57];
                    this->packetTimeStamp[5] = difop_packet_ptr->data[56];
                    this->packetTimeStamp[6] = difop_packet_ptr->data[55];
                    this->packetTimeStamp[7] = difop_packet_ptr->data[54];
                    this->packetTimeStamp[8] = difop_packet_ptr->data[53];
                    this->packetTimeStamp[9] = difop_packet_ptr->data[52];
                } else if(difop_packet_ptr->data[1198] == 0x07 || (difop_packet_ptr->data[1198] & 0x0F) == 4) {
                    fpga_type = 4;
                    time_service_mode_ = difop_packet_ptr->data[45];
                    if (is_msc16 && difop_packet_ptr->data[1198] / 16 == 7 && difop_packet_ptr->data[1202] / 16 == 7) {
                        for (int j = 0; j < 16; ++j) {
                            int adjust_angle_ = difop_packet_ptr->data[680 + 2 * j] * 256 + difop_packet_ptr->data[681 + 2 * j];
                            adjust_angle_ = adjust_angle_ > 32767 ? adjust_angle_ - 65535 : adjust_angle_;
                            msc16_adjust_angle[j] = static_cast<float>(adjust_angle_) * 0.01f;

                            int offset_angle = difop_packet_ptr->data[712 + 2 * j] * 256 + difop_packet_ptr->data[713 + 2 * j];
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
                        int total_working_time = (difop_packet_ptr->data[105] << 24) + (difop_packet_ptr->data[106] << 16)
                                                + (difop_packet_ptr->data[107] << 8) + difop_packet_ptr->data[108];
                        int less_minus40_degree_working_time = (difop_packet_ptr->data[112] << 16)
                                                            + (difop_packet_ptr->data[113] << 8) +
                                                            difop_packet_ptr->data[114];
                        int minus40_to_minus10_working_time = (difop_packet_ptr->data[115] << 16)
                                                            + (difop_packet_ptr->data[116] << 8) +
                                                            difop_packet_ptr->data[117];
                        int minus10_to_30_working_time = (difop_packet_ptr->data[118] << 16)
                                                        + (difop_packet_ptr->data[119] << 8) + difop_packet_ptr->data[120];
                        int positive30_to_70_working_time = (difop_packet_ptr->data[121] << 16)
                                                            + (difop_packet_ptr->data[122] << 8) + difop_packet_ptr->data[123];
                        int positive70_to_100_working_time = (difop_packet_ptr->data[124] << 16)
                                                            + (difop_packet_ptr->data[125] << 8) + difop_packet_ptr->data[126];
                        
                        LS_INFO << "total working time: " << total_working_time / 60 << " hours: " << total_working_time % 60 << " minutes." << LS_END;
                        LS_INFO << "less than     -40 degrees  working time: " << less_minus40_degree_working_time / 60 << " hours: " << less_minus40_degree_working_time % 60 << "minutes" << LS_END;
                        LS_INFO << "-40 degrees ~ -10 degrees  working time: " << minus40_to_minus10_working_time / 60 << " hours: " << minus40_to_minus10_working_time % 60 << "minutes" << LS_END;
                        LS_INFO << "-10 degrees ~ 30  degrees  working time: " << minus10_to_30_working_time / 60 << " hours: " << minus10_to_30_working_time % 60 << "minutes" << LS_END;
                        LS_INFO << "30  degrees ~ 70  degrees  working time: " << positive30_to_70_working_time / 60 << " hours: " << positive30_to_70_working_time % 60 << "minutes" << LS_END;
                        LS_INFO << "70  degrees ~ 100 degrees  working time: " << positive70_to_100_working_time / 60 << " hours: " << positive70_to_100_working_time % 60 << "minutes" << LS_END;
                        is_print_working_time = false;
                    }
                }
                if(fpga_type == 0) {
                    LS_ERROR << "Unknown lidar version" << LS_END;
                } else if (fpga_type != 0 && !start_process_msop_) {
                    determineLidarType();
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
        // output_scan.scan_time = scan_time_;
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
        std::unique_lock<std::mutex> lock(pointcloud_lock);

        sensor_msgs::msg::PointCloud2 pc_msg;
        if (pcl_type) {
            if (point_cloud_xyzi_bak_->points.size() < 100) {
                return;
            }
            pcl::toROSMsg(*point_cloud_xyzi_bak_, pc_msg);
        } else {
            if (point_cloud_xyzirt_bak_->points.size() < 100) {
                return;
            }
            pcl::toROSMsg(*point_cloud_xyzirt_bak_, pc_msg);
        }

        pc_msg.header.frame_id = frame_id;
        pc_msg.header.stamp = rclcpp::Time(sweep_end_time * 1e9);
        pointcloud_pub->publish(std::move(pc_msg));

        return;
    }
    
    void LslidarDriver::publishScan() {
        std::unique_lock<std::mutex> lock(pointcloud_lock);
        scan_msg_bak->header.frame_id = frame_id;
        scan_msg_bak->header.stamp = rclcpp::Time(sweep_end_time * 1e9);
        scan_pub->publish(std::move(scan_msg_bak));
    }

    void LslidarDriver::decodePacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) {
        //couputer azimuth angle for each firing
        for (size_t b_idx = 0; b_idx < BLOCKS_PER_PACKET; ++b_idx) {
            firings.firing_azimuth[b_idx] = (packet->data[b_idx * 100 + 2] + (packet->data[b_idx * 100 + 3] << 8)) % 36000;
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

    bool LslidarDriver::poll() {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet(new lslidar_msgs::msg::LslidarPacket());
        // Since the rslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            int rc = msop_input_->getPacket(packet);
            if (rc == 0) {
                break;
            } else {
                start_process_msop_ = false;
                return false;
            }
        }

        //check if the packet is valid
        if (!checkPacketValidity(packet) || !start_process_msop_) return false;
        packet_num++;
        // packet timestamp
        if (use_time_service && 4 == fpga_type) {
            if (time_service_mode_ == 1) {    //ptp授时
                uint32_t timestamp_s = packet->data[1201] * pow(2, 32) + packet->data[1202] * pow(2, 24) +
                                       packet->data[1203] * pow(2, 16) + packet->data[1204] * pow(2, 8) + 
                                       packet->data[1205];
                uint32_t timestamp_nsce = packet->data[1206] +
                                          packet->data[1207] * pow(2, 8) +
                                          packet->data[1208] * pow(2, 16) +
                                          packet->data[1209] * pow(2, 24); //ns
                timeStamp = rclcpp::Time(timestamp_s, timestamp_nsce);// s,ns
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
            } else if (time_service_mode_ == 0){          //gps授时
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_year = packet->data[1200] + 2000 - 1900;
                cur_time.tm_mon  = packet->data[1201] - 1;
                cur_time.tm_mday = packet->data[1202];
                cur_time.tm_hour = packet->data[1203];
                cur_time.tm_min  = packet->data[1204];
                cur_time.tm_sec  = packet->data[1205];
                packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = packet->data[1206] +
                                 packet->data[1207] * pow(2, 8) +
                                 packet->data[1208] * pow(2, 16) +
                                 packet->data[1209] * pow(2, 24); //ns
                timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
            }else if (time_service_mode_ == 2){          //ntp授时
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_year = packet->data[1200] + 2000 - 1900;
                cur_time.tm_mon  = packet->data[1201] - 1;
                cur_time.tm_mday = packet->data[1202];
                cur_time.tm_hour = packet->data[1203];
                cur_time.tm_min  = packet->data[1204];
                cur_time.tm_sec  = packet->data[1205];
                packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = (packet->data[1206] +
                                 packet->data[1207] * pow(2, 8) +
                                 packet->data[1208] * pow(2, 16) +
                                 packet->data[1209] * pow(2, 24)) * 1e3; //ns
                timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
            }
        } else if (use_time_service && 3 == fpga_type) { // gps授时
            memset(&cur_time, 0, sizeof(cur_time));
            cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
            cur_time.tm_mon  = this->packetTimeStamp[8] - 1;
            cur_time.tm_mday = this->packetTimeStamp[7];
            cur_time.tm_hour = this->packetTimeStamp[6];
            cur_time.tm_min  = this->packetTimeStamp[5];
            cur_time.tm_sec  = this->packetTimeStamp[4];
            packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
            packet_time_ns = (packet->data[1200] +
                              packet->data[1201] * pow(2, 8) +
                              packet->data[1202] * pow(2, 16) +
                              packet->data[1203] * pow(2, 24)) * 1e3; //ns
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
            packet->stamp = this->get_clock()->now();
            current_packet_time = this->get_clock()->now().seconds();
        }

        //decode the packet
        decodePacket(packet);
        // find the start of a new revolution
        // if there is one, new_sweep_start will be the index of the start firing,
        // otherwise, new_sweep_start will be FIRINGS_PER_PACKET.
        size_t new_sweep_start = 0;
        if (packet_num > number_threshold) {
            do {
                if (abs(firings.azimuth[new_sweep_start] - last_azimuth) > 35000) {
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

        if (lidar_type == "c32_3") {
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

        for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
            if (angle_change) {
                if ("C32W" == lidar_type) {
                    if (fir_idx % 32 == 29 || fir_idx % 32 == 6 || fir_idx % 32 == 14 || fir_idx % 32 == 22 ||
                        fir_idx % 32 == 30 || fir_idx % 32 == 7 || fir_idx % 32 == 15 || fir_idx % 32 == 23) {
                        firings.azimuth[fir_idx] += 389;
                    }
                    if (firings.azimuth[fir_idx] >= 36000) firings.azimuth[fir_idx] -= 36000;
                }
                if ("MSC16" == lidar_type) {
                    firings.azimuth[fir_idx] += msc16_offset_angle[fir_idx % 16] < 0 ?
                                                msc16_offset_angle[fir_idx % 16] + 36000 : msc16_offset_angle[fir_idx % 16];
                    if (firings.azimuth[fir_idx] >= 36000) { firings.azimuth[fir_idx] -= 36000; }
                }
            }
            //check if the point is valid
            if (!(firings.distance[fir_idx] >= min_range && firings.distance[fir_idx] <= max_range)) continue;
            if (angle_able_max > 36000){
                if((firings.azimuth[fir_idx] > (angle_able_max - 36000)) && firings.azimuth[fir_idx] < angle_able_min) continue;
            } else {
                if((firings.azimuth[fir_idx] > angle_able_max ) || firings.azimuth[fir_idx] < angle_able_min) continue;
            }
            //convert the point to xyz coordinate
            size_t table_idx = firings.azimuth[fir_idx];
            double cos_azimuth = cos_azimuth_table[table_idx];
            double sin_azimuth = sin_azimuth_table[table_idx];
            double x_coord, y_coord, z_coord;

            if (coordinate_opt) {
                int tmp_idx = conversionAngle - firings.azimuth[fir_idx] < 0 ?
                              conversionAngle - firings.azimuth[fir_idx] + 36000 : conversionAngle - firings.azimuth[fir_idx];
                                                                                
                x_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * cos_azimuth +
                          R1 * cos_azimuth_table[tmp_idx];
                y_coord = -firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * sin_azimuth +
                          R1 * sin_azimuth_table[tmp_idx];
                z_coord = firings.distance[fir_idx] * sin_scan_altitude[fir_idx % lidar_number_];
            } else {
                //Y-axis correspondence 0 degree
                int tmp_idx = firings.azimuth[fir_idx] - conversionAngle < 0 ?
                              firings.azimuth[fir_idx] - conversionAngle + 36000 : firings.azimuth[fir_idx] - conversionAngle;
                                                                                
                x_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * sin_azimuth +
                          R1 * sin_azimuth_table[tmp_idx];
                y_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * cos_azimuth +
                          R1 * cos_azimuth_table[tmp_idx];
                z_coord = firings.distance[fir_idx] * sin_scan_altitude[fir_idx % lidar_number_];
            }

            // computer the time of the point
            double time;
            if (last_packet_time > 1e-6) {
                time = current_packet_time - sweep_end_time - (current_packet_time - last_packet_time) *
                       (SCANS_PER_PACKET - fir_idx - 1) / SCANS_PER_PACKET;
            } else {
                time = current_packet_time;
            }
            
            remapped_scan_idx = 0;
            switch (ring_) {
                case 33:    // CH32RN  C32WB
                    remapped_scan_idx = (fir_idx % 32) % 4 * 16 >= 24 ? (fir_idx % 32) % 4 * 16 - 24 + fir_idx % 32 / 4 : (fir_idx % 32) % 4 * 16 + fir_idx % 32 / 4;
                    break;
                case 32:
                    remapped_scan_idx = (fir_idx % 32) % 4 * 8 + fir_idx % 32 / 4;
                    break;
                case 31:
                    remapped_scan_idx = (fir_idx % 32) % 2 == 0 ? (fir_idx % 32) / 2 : (fir_idx % 32) / 2 + 16;
                    break;     
                case 17:
                    remapped_scan_idx = (fir_idx % 16) / 4 + (fir_idx % 16) % 4 * 4; //c16 4.0 国产化
                    break;
                case 16:
                    remapped_scan_idx = (fir_idx % 16) % 2 == 0 ? (fir_idx % 16) / 2 : (fir_idx % 16) / 2 + 8;
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
                    if (coordinate_opt) {
                        float horizontal_angle = firings.azimuth[fir_idx] * 0.01f * DEG_TO_RAD;
                        uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                        point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                        scan_msg->ranges[point_size - point_index - 1] = firings.distance[fir_idx];
                        scan_msg->intensities[point_size - point_index - 1] = firings.intensity[fir_idx];
                    } else {
                        float h_angle = (45000.0 - firings.azimuth[fir_idx]) < 36000.0 ?
                                        45000.0 - firings.azimuth[fir_idx] : 9000.0 - firings.azimuth[fir_idx];
                        float horizontal_angle = h_angle * 0.01f * DEG_TO_RAD;

                        //float horizontal_angle = firings.azimuth[fir_idx] * 0.01f * DEG_TO_RAD;
                        uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                        point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                        scan_msg->ranges[point_index] = firings.distance[fir_idx];
                        scan_msg->intensities[point_index] = firings.intensity[fir_idx];
                    }
                }
            }
        }
        // a new sweep begins ----------------------------------------------------
        if (end_fir_idx != SCANS_PER_PACKET) {
            //publish Last frame scan
            if (last_packet_time > 1e-6) {
                sweep_end_time = current_packet_time - (current_packet_time - last_packet_time) *
                                 (SCANS_PER_PACKET - end_fir_idx) / SCANS_PER_PACKET;
            } else {
                sweep_end_time = current_packet_time;
            }

            sweep_end_time = sweep_end_time > 0 ? sweep_end_time : 0;

            {
                std::unique_lock <std::mutex> lock(pointcloud_lock);
                point_cloud_xyzirt_bak_ = std::move(point_cloud_xyzirt_);
                point_cloud_xyzi_bak_ = std::move(point_cloud_xyzi_);
                scan_msg_bak = std::move(scan_msg);
            }

            std::thread pointcloud_pub_thread([this] { publishPointcloud(); });
            pointcloud_pub_thread.detach();

            if (publish_scan) {
                std::thread laserscan_pub_thread(&LslidarDriver::publishScan, this);
                laserscan_pub_thread.detach();
            };

            point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>());
            point_cloud_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            point_cloud_xyzirt_->header.frame_id = frame_id;
            point_cloud_xyzirt_->height = 1;

            point_cloud_xyzi_->header.frame_id = frame_id;
            point_cloud_xyzi_->height = 1;

            if (publish_scan) {
                //scan_msg.reset(new sensor_msgs::msg::LaserScan());
                scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
                scan_msg->angle_min = 0;
                scan_msg->angle_max = M_PI * 2;
                scan_msg->range_min = min_range;
                scan_msg->range_max = max_range;
                scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
                point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

                scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
            }

            //prepare the next frame scan
            //sweep_end_time = packet->stamp.toSec() - (end_fir_idx - start_fir_idx) * 3.125 * 1e-6;
            last_azimuth = firings.azimuth[SCANS_PER_PACKET - 1];
            start_fir_idx = end_fir_idx;
            end_fir_idx = SCANS_PER_PACKET;
            for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
                if (angle_change) {
                    if ("C32W" == lidar_type) {
                        if (fir_idx % 32 == 29 || fir_idx % 32 == 6 || fir_idx % 32 == 14 || fir_idx % 32 == 22 ||
                            fir_idx % 32 == 30 || fir_idx % 32 == 7 || fir_idx % 32 == 15 || fir_idx % 32 == 23) {
                            firings.azimuth[fir_idx] += 389;
                        }
                        if (firings.azimuth[fir_idx] >= 36000) firings.azimuth[fir_idx] -= 36000;
                    }
                    if ("MSC16" == lidar_type) {
                        firings.azimuth[fir_idx] += msc16_offset_angle[fir_idx % 16] < 0 ?
                                                    msc16_offset_angle[fir_idx % 16] + 36000 : msc16_offset_angle[fir_idx % 16];
                        if (firings.azimuth[fir_idx] >= 36000) { firings.azimuth[fir_idx] -= 36000; }
                    }
                }

                //check if the point is valid
                if (!(firings.distance[fir_idx] >= min_range && firings.distance[fir_idx] <= max_range)) continue;
                if (angle_able_max > 36000){
                    if((firings.azimuth[fir_idx] > (angle_able_max - 36000)) && firings.azimuth[fir_idx] < angle_able_min) continue;
                } else {
                    if((firings.azimuth[fir_idx] > angle_able_max ) || firings.azimuth[fir_idx] < angle_able_min) continue;
                }

                //convert the point to xyz coordinate
                size_t table_idx = firings.azimuth[fir_idx];
                double cos_azimuth = cos_azimuth_table[table_idx];
                double sin_azimuth = sin_azimuth_table[table_idx];
                double x_coord, y_coord, z_coord;

                if (coordinate_opt) {
                    int tmp_idx = conversionAngle - firings.azimuth[fir_idx] < 0 ?
                                  conversionAngle - firings.azimuth[fir_idx] + 36000 : conversionAngle - firings.azimuth[fir_idx];
                                                                                    
                    x_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * cos_azimuth +
                              R1 * cos_azimuth_table[tmp_idx];
                    y_coord = -firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * sin_azimuth +
                              R1 * sin_azimuth_table[tmp_idx];
                    z_coord = firings.distance[fir_idx] * sin_scan_altitude[fir_idx % lidar_number_];
                } else {
                    //Y-axis correspondence 0 degree
                    int tmp_idx = firings.azimuth[fir_idx] - conversionAngle < 0 ?
                                  firings.azimuth[fir_idx] - conversionAngle + 36000 : firings.azimuth[fir_idx] - conversionAngle;
                                                                                    
                    x_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * sin_azimuth +
                              R1 * sin_azimuth_table[tmp_idx];
                    y_coord = firings.distance[fir_idx] * cos_scan_altitude[fir_idx % lidar_number_] * cos_azimuth +
                              R1 * cos_azimuth_table[tmp_idx];
                    z_coord = firings.distance[fir_idx] * sin_scan_altitude[fir_idx % lidar_number_];
                }

                // computer the time of the point
                double time;
                if (last_packet_time > 1e-6) {
                    // time = current_packet_time - (current_packet_time - last_packet_time) *
                    //        (SCANS_PER_PACKET - fir_idx - 1) / SCANS_PER_PACKET;         
                    time = (current_packet_time - last_packet_time) * (fir_idx - start_fir_idx) / SCANS_PER_PACKET;
                } else {
                    time = current_packet_time;
                }

                remapped_scan_idx = 0;
                switch (ring_) {
                    case 33:    // CH32RN  C32WB
                        remapped_scan_idx = (fir_idx % 32) % 4 * 16 >= 24 ? (fir_idx % 32) % 4 * 16 - 24 + fir_idx % 32 / 4 : (fir_idx % 32) % 4 * 16 + fir_idx % 32 / 4;
                        break;
                    case 32:
                        remapped_scan_idx = (fir_idx % 32) % 4 * 8 + fir_idx % 32 / 4;
                        break;
                    case 31:
                        remapped_scan_idx = (fir_idx % 32) % 2 == 0 ? (fir_idx % 32) / 2 : (fir_idx % 32) / 2 + 16;
                        break;     
                    case 17:
                        remapped_scan_idx = (fir_idx % 16) / 4 + (fir_idx % 16) % 4 * 4; //c16 4.0 国产化
                        break;
                    case 16:
                        remapped_scan_idx = (fir_idx % 16) % 2 == 0 ? (fir_idx % 16) / 2 : (fir_idx % 16) / 2 + 8;
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
                        if (coordinate_opt) {
                            float horizontal_angle = firings.azimuth[fir_idx] * 0.01f * DEG_TO_RAD;
                            uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                            point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                            scan_msg->ranges[point_size - point_index - 1] = firings.distance[fir_idx];
                            scan_msg->intensities[point_size - point_index - 1] = firings.intensity[fir_idx];
                        } else {
                            float h_angle = (45000.0 - firings.azimuth[fir_idx]) < 36000.0 ?
                                            45000.0 - firings.azimuth[fir_idx] : 9000.0 - firings.azimuth[fir_idx];
                            float horizontal_angle = h_angle * 0.01f * DEG_TO_RAD;
                            uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                            point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                            scan_msg->ranges[point_index] = firings.distance[fir_idx];
                            scan_msg->intensities[point_index] = firings.intensity[fir_idx];
                        }
                    }
                }
            }
        }

        last_packet_time = current_packet_time;
        
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
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet0(new lslidar_msgs::msg::LslidarPacket());
        packet0->data[0] = 0x00;
        packet0->data[1] = 0x00;
        int rc_msop = -1;

        if(!is_get_difop_) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        if (req->laser_control == 1) {
            if ((rc_msop = msop_input_->getPacket(packet0)) == 0) {
                res->result = 1;
                LS_WARN << "receive cmd: " << req->laser_control <<" already power on status" << LS_END;
                return true;
            }
            LS_MSG << "receive cmd: " << req->laser_control << ", power on" << LS_END;
            if (4 == fpga_type) {
                config_data[50] = 0xBB;
            } else if (3 == fpga_type) {
                config_data[8] = 0x02;
                config_data[9] = 0x58;
                config_data[45] = 0x00; 
            }
            sendPacketTolidar(config_data);
            double time1 = this->get_clock()->now().seconds();

            do {
                rc_msop = msop_input_->getPacket(packet0);
                double time2 = this->get_clock()->now().seconds();
                if (time2 - time1 > 20) {
                    res->result = 0;
                    LS_WARN << "lidar connect error" <<LS_END;
                    return true;
                }
            } while ((rc_msop != 0) && (packet0->data[0] != 0xff) && (packet0->data[1] != 0xee));
            sleep(3);
            res->result = 1;
        } else if (req->laser_control == 0) {
            if (4 == fpga_type) {
                config_data[50] = 0xAA;
            } else if (3 == fpga_type) {
                config_data[45] = 0x01;
            }
            LS_MSG << "receive cmd: " << req->laser_control << ", power off" << LS_END;
            sendPacketTolidar(config_data);
            res->result = 1;
        } else {
            LS_ERROR << "cmd error" << LS_END;
            res->result = 0;
        }
        return true;
    }

    bool LslidarDriver::timeService(std::shared_ptr <lslidar_msgs::srv::TimeService::Request> req,
                                    std::shared_ptr <lslidar_msgs::srv::TimeService::Response> res) {
        LS_INFO << "Start to modify lidar time service mode" << LS_END;
        if(!is_get_difop_) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        std::string time_service_mode = req->time_service_mode;
        transform(time_service_mode.begin(), time_service_mode.end(), time_service_mode.begin(), ::tolower);

        if (time_service_mode == "gps") {
            config_data[45] = 0x00;
        } else if (4 == fpga_type && time_service_mode == "ptp") {
            config_data[45] = 0x01;
        } else if (4 == fpga_type && time_service_mode == "ntp") {
            config_data[45] = 0x02;
            std::string ntp_ip = req->ntp_ip;
            std::regex ipv4(
                    "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
            if (!regex_match(ntp_ip, ipv4)) {
                LS_ERROR << "Parameter error, please check the input parameters" << LS_END;
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
            LS_ERROR << "Parameter error, please check the input parameters." << LS_END;
            res->result = false;
            return true;
        }
 
        res->result = true;
        sendPacketTolidar(config_data);
        LS_MSG << "The current timing method is " << time_service_mode << ". Time service method modified successfully!" << LS_END;;
        return true;
    }

    bool LslidarDriver::motorControl(std::shared_ptr<lslidar_msgs::srv::MotorControl::Request> req,
                                     std::shared_ptr<lslidar_msgs::srv::MotorControl::Response> res) {
        if(!is_get_difop_) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        if (req->motor_control == 1) {
            if (3 == fpga_type) {
            	config_data[8] = 0x02;
                config_data[9] = 0x58;
            }
            config_data[41] = 0x00;
        } else if (req->motor_control == 0) {
            config_data[41] = 0x01;
        } else {
            LS_ERROR << "Parameter error, please check the input parameters" << LS_END;
            res->result = false;
            return true;
        }

        res->result = true;
        sendPacketTolidar(config_data);
        LS_MSG << "Motor control: " << req->motor_control << ", set successfully!" << LS_END;
        return true;
    }

    bool LslidarDriver::motorSpeed(std::shared_ptr <lslidar_msgs::srv::MotorSpeed::Request> req,
                                   std::shared_ptr <lslidar_msgs::srv::MotorSpeed::Response> res) {
        if(!is_get_difop_) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
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
            LS_ERROR << "Parameter error, please check the input parameters" << LS_END;
            res->result = false;
            return true;
        }
        res->result = true;
        sendPacketTolidar(config_data);
        LS_MSG << "Motor speed: " << req->motor_speed << " HZ, set successfully!" << LS_END;
        return true;
    }

    bool LslidarDriver::setDataPort(std::shared_ptr <lslidar_msgs::srv::DataPort::Request> req,
                                    std::shared_ptr <lslidar_msgs::srv::DataPort::Response> res) {
        if(!is_get_difop_) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        int dev_port = config_data[26] * 256 + config_data[27];
        if (req->data_port < 1025 || req->data_port > 65535 || req->data_port == dev_port) {
            LS_ERROR << "Parameter error, please check the input parameters" << LS_END;
            res->result = false;
            return true;
        } else {
            config_data[24] = req->data_port / 256;
            config_data[25] = req->data_port % 256;
        }
        res->result = true;
        sendPacketTolidar(config_data);
        LS_MSG << "Set successfully! Current lidar MSOP port:" << req->data_port << LS_END;
        LS_MSG << "Please modify the corresponding parameters in the launch file" << LS_END;
        return true;
    }

    bool LslidarDriver::setDevPort(std::shared_ptr <lslidar_msgs::srv::DevPort::Request> req,
                                   std::shared_ptr <lslidar_msgs::srv::DevPort::Response> res) {
        if(!is_get_difop_) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        int data_port = config_data[24] * 256 + config_data[25];
        if (req->dev_port < 1025 || req->dev_port > 65535 || req->dev_port == data_port) {
            LS_ERROR << "Parameter error, please check the input parameters" << LS_END;
            res->result = false;
            return true;
        } else {
            config_data[26] = req->dev_port / 256;
            config_data[27] = req->dev_port % 256;
        }
        res->result = true;
        sendPacketTolidar(config_data);
        LS_MSG << "Set successfully! Current lidar DIFOP port:" << req->dev_port << LS_END;
        LS_MSG << "Please modify the corresponding parameters in the launch file" << LS_END;
        return true;
    }

    bool LslidarDriver::setDataIp(std::shared_ptr <lslidar_msgs::srv::DataIp::Request> req,
                                  std::shared_ptr <lslidar_msgs::srv::DataIp::Response> res) {
        std::regex ipv4("\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
        if (!regex_match(req->data_ip, ipv4)) {
            LS_ERROR << "Parameter error, please check the input parameters" << LS_END;
            res->result = false;
            return true;
        }

        if(!is_get_difop_) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }
        setPacketHeader(config_data);
        is_get_difop_ = false;

        unsigned short first_value, second_value, third_value, end_value;
        sscanf(req->data_ip.c_str(), "%hu.%hu.%hu.%hu", &first_value, &second_value, &third_value, &end_value);

        std::string destination_ip = std::to_string(config_data[14]) + "." + std::to_string(config_data[15]) + "." +
                                     std::to_string(config_data[16]) + "." + std::to_string(config_data[17]);
        if (first_value == 0 || first_value == 127 ||
            (first_value >= 224 && first_value <= 255) || destination_ip == req->data_ip) {
            LS_ERROR << "Parameter error, please check the input parameters" << LS_END;
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
        LS_MSG << "Set successfully! Current lidar ip:" << req->data_ip.c_str() << LS_END;
        LS_MSG << "Please modify the corresponding parameters in the launch file" << LS_END;
        return true;
    }

    bool LslidarDriver::setDestinationIp(std::shared_ptr <lslidar_msgs::srv::DestinationIp::Request> req,
                                         std::shared_ptr <lslidar_msgs::srv::DestinationIp::Response> res) {
        std::regex ipv4("\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
        if (!regex_match(req->destination_ip, ipv4)) {
            LS_ERROR << "Parameter error, please check the input parameters" << LS_END;
            res->result = false;
            return true;
        }

        if(!is_get_difop_) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_ERROR << "Can not get dev packet! Set failed!" << LS_END;
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
            LS_ERROR << "Parameter error, please check the input parameters" << LS_END;
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
        LS_MSG << "Set successfully! Current lidar destination ip: " << req->destination_ip.c_str() << LS_END;
        LS_MSG << "Please modify the local IP address to " << req->destination_ip.c_str() << LS_END;
        return true;
    }

    bool LslidarDriver::determineLidarType(){
        lslidar_msgs::msg::LslidarPacket::UniquePtr pkt(new lslidar_msgs::msg::LslidarPacket);

        while (true) {
            int rc_ = msop_input_->getPacket(pkt);
            if (rc_ == 0) break;
            if (rc_ < 0) return false;
        }

        if (pkt->data[1211] == 0x01) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c1_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c1_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 1;
            lidar_number_ = 1;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            number_threshold = 1;
            lidar_type = "C1";
            LS_INFO << "lidar type: C1" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if (pkt->data[1211] == 0x03) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c1_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c1_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 1;
            lidar_number_ = 1;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            number_threshold = 1;
            lidar_type = "C1P";
            LS_INFO << "lidar type: C1P" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if(pkt->data[1211] == 0x06){
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c1_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c1_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 1;
            lidar_number_ = 1;
            distance_unit = 0.1;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            number_threshold = 1;
            lidar_type = "N301";
            LS_INFO << "lidar type: N301" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if (pkt->data[1211] == 0x07) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c8f_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c8f_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 8;
            lidar_number_ = 8;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            number_threshold = 2;
            lidar_type = "C8F";         
            LS_INFO << "lidar type: C8F" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if (pkt->data[1211] == 0x08) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(c8_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c8_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 8;
            lidar_number_ = 8;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            number_threshold = 2;
            lidar_type = "C8";               
            LS_INFO << "lidar type: C8" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if (pkt->data[1211] == 0x09) {
            for (int i = 0; i < 8; ++i) {
                sin_scan_altitude[i] = sin(ckm8_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(ckm8_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 8;
            lidar_number_ = 8;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            number_threshold = 2;
            lidar_type = "CKM8";
            LS_INFO << "lidar type: CKM8/C4" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if (pkt->data[1211] == 0x10) {
            for (int i = 0; i < 16; ++i) {
                sin_scan_altitude[i] = sin(c16_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c16_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 16;
            lidar_number_ = 16;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_type = "C16";
            LS_INFO << "lidar type: C16" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
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
            lidar_type = "MSC16";        
            LS_INFO << "lidar type: MSC16" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if (pkt->data[1211] == 0x12) {
            for (int i = 0; i < 16; ++i) {
                sin_scan_altitude[i] = sin(c16_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c16_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 17;
            lidar_number_ = 16;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_type = "C16_domestic";
            LS_INFO << "lidar type: C16 domestic" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if (pkt->data[1211] == 0x20) {
            for (int i = 0; i < 32; ++i) {
                sin_scan_altitude[i] = sin(c32_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i] = cos(c32_vertical_angle[i] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_;
            conversionAngle = conversionAngle_;
            lidar_type = "C32";
            LS_INFO << "lidar type: C32" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if (pkt->data[1211] == 0x45) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32wp_vertical_angle[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32wp_vertical_angle[k] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_C32W;
            conversionAngle = conversionAngle_;                
            lidar_type = "C32WP";
            LS_INFO << "lidar type: C32WP" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
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
            lidar_type = "C32W";
            LS_INFO << "lidar type: C32W" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if (pkt->data[1211] == 0x47) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32wn_vertical_angle2[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32wn_vertical_angle2[k] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_C32W;
            conversionAngle = conversionAngle_;
            lidar_type = "C32WN";
            LS_INFO << "lidar type: C32WN" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if (pkt->data[1211] == 0x48) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32wb_vertical_angle2[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32wb_vertical_angle2[k] * DEG_TO_RAD);
            }
            ring_ = 33;
            lidar_number_ = 32;
            R1 = R1_C32W;
            conversionAngle = conversionAngle_;
            lidar_type = "C32WB";
            LS_INFO << "lidar type: C32WB" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if (pkt->data[1211] == 0x5a) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32_90_vertical_angle[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32_90_vertical_angle[k] * DEG_TO_RAD);
            }
            ring_ = 32;
            lidar_number_ = 32;
            R1 = R1_90;
            conversionAngle = conversionAngle_90;
            lidar_type = "CH32R";
            LS_INFO << "lidar type: CH32R" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
        } else if (pkt->data[1211] == 0x5d) {
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k] = sin(c32rn_vertical_angle[k] * DEG_TO_RAD);
                cos_scan_altitude[k] = cos(c32rn_vertical_angle[k] * DEG_TO_RAD);
            }
            ring_ = 33;
            lidar_number_ = 32;
            R1 = R1_90;
            conversionAngle = conversionAngle_90;
            lidar_type = "CH32RN";
            LS_INFO << "lidar type: CH32R" << LS_END;
            if (pkt->data[1210] == 0x39)  return_mode = 2;
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
            lidar_type = "c16_3";
            LS_INFO << "lidar type: C16, version 3.0" << LS_END;
            if (pkt->data[1204] == 0x39) return_mode = 2;
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
            lidar_type = "c32_3";
            LS_INFO << "lidar type: C32, version 3.0" << LS_END;
            if (pkt->data[1204] == 0x39) return_mode = 2;
        } else {
            LS_ERROR << "Unknown lidar model,please check lidar model" << LS_END;;
            // rclcpp::shutdown();
            return false;
        }
        LS_INFO <<"return mode: " << return_mode << LS_END;;
        start_process_msop_ = true;

        return true;
    }

    void LslidarDriver::pollThread(void) {
        while (rclcpp::ok()) {
            poll();
        }
    }

}  // namespace lslidar_driver

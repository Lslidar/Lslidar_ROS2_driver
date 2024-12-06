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

#include "lslidar_ch_driver/lslidar_driver.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace lslidar_ch_driver {
    LslidarChDriver::LslidarChDriver() : LslidarChDriver(rclcpp::NodeOptions()) {
        return;
    }

    LslidarChDriver::LslidarChDriver(
            const rclcpp::NodeOptions &options) :
            rclcpp::Node("lslidar_node", options),
            threadPool_(std::make_unique<ThreadPool>(2)),
            socket_id(-1),
            gain_prism_angle(true),
            point_cloud_xyzirt_(new pcl::PointCloud<VPoint>),
            point_cloud_xyzirt_bak_(new pcl::PointCloud<VPoint>),
            scan_msg(new sensor_msgs::msg::LaserScan()),
            scan_msg_bak(new sensor_msgs::msg::LaserScan()),
            time_service_mode("system") {
        prism_offset = 0.0;
        last_packet_timestamp = 0.0;
        is_update_difop_packet = false;
        for (int j = 0; j < 36000; ++j) {
            sin_list[j] = sin(j * 0.01 * DEG_TO_RAD);
            cos_list[j] = cos(j * 0.01 * DEG_TO_RAD);
        }

        return;
    }

    LslidarChDriver::~LslidarChDriver() {
        if (difop_thread_ && difop_thread_->joinable()) {
            difop_thread_->join();  // 等待线程完成
        }
        return;
    }

    bool LslidarChDriver::loadParameters() {
        this->declare_parameter<std::string>("lidar_type", "cx128s2");
        this->declare_parameter<std::string>("device_ip", "192.168.1.200");
        this->declare_parameter<int>("msop_port", 2368);
        this->declare_parameter<int>("difop_port", 2369);
        this->declare_parameter<bool>("add_multicast", false);
        this->declare_parameter<std::string>("group_ip", "234.2.3.2");
        this->declare_parameter<bool>("use_time_service", false);
        this->declare_parameter<double>("min_range", 0.3);
        this->declare_parameter<double>("max_range", 150.0);
        this->declare_parameter<int>("scan_start_angle", 0);
        this->declare_parameter<int>("scan_end_angle", 18000);
        this->declare_parameter<std::string>("frame_id", "laser_link");
        this->declare_parameter<std::string>("topic_name", "lslidar_point_cloud");
        this->declare_parameter<int>("echo_num", 0);
        this->declare_parameter<bool>("publish_scan", false);
        this->declare_parameter<int>("channel_num", 15);
        this->declare_parameter<double>("horizontal_angle_resolution", 0.2);
        this->declare_parameter<double>("packet_rate", 1695.0);
        this->declare_parameter<std::string>("pcap", "");

        this->get_parameter("lidar_type", lidar_type);
        this->get_parameter("device_ip", lidar_ip_string);
        this->get_parameter("msop_port", msop_udp_port);
        this->get_parameter("difop_port", difop_udp_port);
        this->get_parameter("add_multicast", add_multicast);
        this->get_parameter("group_ip", group_ip_string);
        this->get_parameter("use_time_service", use_time_service);
        this->get_parameter("min_range", min_range);
        this->get_parameter("max_range", max_range);
        this->get_parameter("scan_start_angle", scan_start_angle);
        this->get_parameter("scan_end_angle", scan_end_angle);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("topic_name", pointcloud_topic);
        this->get_parameter("echo_num", echo_num);
        this->get_parameter("publish_scan", publish_laserscan);
        this->get_parameter("channel_num", channel_num);
        this->get_parameter("horizontal_angle_resolution", horizontal_angle_resolution);
        this->get_parameter("packet_rate", packet_rate);
        this->get_parameter("pcap", dump_file);
        
        return true;
    }

    void LslidarChDriver::outputParameters() {
        LS_PARAM << "******** CH ROS2 driver version: 1.0.3 ********" << LS_END;
        LS_PARAM << "lidar_type: " << lidar_type << LS_END;
        LS_PARAM << "dump file: " << dump_file << LS_END;
        LS_PARAM << "packet rate: " << packet_rate << LS_END;
        LS_PARAM << "add multicast: " << std::boolalpha << add_multicast << LS_END;
        LS_PARAM << "use time service: " << std::boolalpha << use_time_service << LS_END;
        LS_PARAM << "min range: " << min_range << LS_END;
        LS_PARAM << "max range: "<< max_range << LS_END;
        LS_PARAM << "scan start angle: " << scan_start_angle << LS_END;
        LS_PARAM << "scan end angle: " << scan_end_angle << LS_END;
        LS_PARAM << "horizontal angle resolution: " << horizontal_angle_resolution << LS_END;
        LS_PARAM << "frame id: " << frame_id << LS_END;
        LS_PARAM << "pointcloud topic: " << pointcloud_topic << LS_END;
        LS_PARAM << "publish laserscan: " << std::boolalpha << publish_laserscan << LS_END;
        LS_PARAM << "echo num: " << echo_num << LS_END;
        
        if (publish_laserscan) {
            LS_PARAM << "laserscan channel num: " << channel_num << LS_END;
            if (channel_num < 0) {
                channel_num = 0;
                LS_WARN <<"channel_num outside of the index, select channel 0 instead!" << LS_END;
            } else if (channel_num > maxChannelNumbers[lidar_type]) {
                channel_num = maxChannelNumbers[lidar_type] - 1;
                LS_WARN <<"channel_num outside of the index, select channel " <<  channel_num << " instead!" << LS_END;
            }
            LS_PARAM << "select channel num: " << channel_num << LS_END;
        }

        LS_SOCKET << "Accepting packets from IP address: " << lidar_ip_string.c_str() << LS_END;
        if (add_multicast) LS_SOCKET << "Opening UDP socket: group address: " << group_ip_string.c_str() << LS_END;
        LS_SOCKET << "Opening UDP socket msop port: " << msop_udp_port << LS_END;
        LS_SOCKET << "Opening UDP socket difop port: " << difop_udp_port << LS_END;
    }

    bool LslidarChDriver::createRosIO() {
        // ROS diagnostics
        pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
        laserscan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        if (dump_file != "") {
            if (lidar_type == "CX1S3" || lidar_type == "CX6S3" || lidar_type == "CH16X1" || lidar_type == "CH128S1" || 
                lidar_type == "CX128S2" || lidar_type == "CX126S3" || lidar_type == "CH256") {
                msop_input_.reset(new lslidar_ch_driver::InputPCAP(this, msop_udp_port, 1212, packet_rate, dump_file));
                difop_input_.reset(new lslidar_ch_driver::InputPCAP(this, difop_udp_port, 1206, 1, dump_file));
            } else if (lidar_type == "CH64W" || lidar_type == "CB64S1-A" || lidar_type == "CH128X1") {
                msop_input_.reset(new lslidar_ch_driver::InputPCAP(this, msop_udp_port, 1206, packet_rate, dump_file));
                difop_input_.reset(new lslidar_ch_driver::InputPCAP(this, difop_udp_port, 1206, 1, dump_file));
                echo_byte = 1205;
            } else {
                LS_ERROR << "Unknown lidar model,please check lidar model." << LS_END;
                return false;
            }
        } else {
            if (lidar_type == "CX1S3" || lidar_type == "CX6S3" || lidar_type == "CH16X1" || lidar_type == "CH128S1" || 
                lidar_type == "CX128S2" || lidar_type == "CX126S3" || lidar_type == "CH256") {
                msop_input_.reset(new lslidar_ch_driver::InputSocket(this, msop_udp_port, 1212));
                difop_input_.reset(new lslidar_ch_driver::InputSocket(this, difop_udp_port, 1206));
            } else if (lidar_type == "CH64W" || lidar_type == "CB64S1-A" || lidar_type == "CH128X1"){
                msop_input_.reset(new lslidar_ch_driver::InputSocket(this, msop_udp_port, 1206));
                difop_input_.reset(new lslidar_ch_driver::InputSocket(this, difop_udp_port, 1206));
                echo_byte = 1205;
            } else {
                LS_ERROR << "Unknown lidar model,please check lidar model." << LS_END;
                return false;
            }
        }

        difop_thread_ = std::make_shared<std::thread>([this]() { difopPoll(); });
        
        return true;
    }

    bool LslidarChDriver::initialize() {

        this->initTimeStamp();

        initializeMaxChannelNumbers();

        if (!loadParameters()) {
            LS_ERROR << "Cannot load all required ROS parameters..." << LS_END;
            return false;
        }

        outputParameters();

        for (double &j : prism_angle) {
            j = 0.0f;
        }
        // ch64w
        for (int m = 0; m < 128; ++m) {
            //右边
            if (m / 4 % 2 == 0) {
                ch64w_sin_theta_1[m] = sin((-25 + floor(m / 8) * 2.5) * DEG_TO_RAD);
                ch64w_sin_theta_2[m] = sin((m % 4) * 0.35 * DEG_TO_RAD);
                ch64w_cos_theta_1[m] = cos((-25 + floor(m / 8) * 2.5) * DEG_TO_RAD);
                ch64w_cos_theta_2[m] = cos((m % 4) * 0.35 * DEG_TO_RAD);
            } else { //左边
                ch64w_sin_theta_1[m] = sin((-24 + prism_offset + floor(m / 8) * 2.5) * DEG_TO_RAD);
                ch64w_sin_theta_2[m] = sin((m % 4) * 0.35 * DEG_TO_RAD);
                ch64w_cos_theta_1[m] = cos((-24 + prism_offset + floor(m / 8) * 2.5) * DEG_TO_RAD);
                ch64w_cos_theta_2[m] = cos((m % 4) * 0.35 * DEG_TO_RAD);
            }
        }

        for (int i = 0; i < 128; i++) {
            // 左边
            if (i / 4 % 2 == 0) {
                sin_theta_1[i] = sin((big_angle[i / 4] + prism_offset) * DEG_TO_RAD);
                cos_theta_1[i] = cos((big_angle[i / 4] + prism_offset) * DEG_TO_RAD);

            } else {
                sin_theta_1[i] = sin(big_angle[i / 4] * DEG_TO_RAD);
                cos_theta_1[i] = cos(big_angle[i / 4] * DEG_TO_RAD);
            }

            if (lidar_type == "CH128S1" || lidar_type == "CX128S2") {
                sin_theta_1[i] = sin(big_angle_ch128s1[i / 4] * DEG_TO_RAD);
                cos_theta_1[i] = cos(big_angle_ch128s1[i / 4] * DEG_TO_RAD);
            }
            sin_theta_2[i] = sin((i % 4) * (-0.17) * DEG_TO_RAD);
            cos_theta_2[i] = cos((i % 4) * (-0.17) * DEG_TO_RAD);
        }

        for (int l = 0; l < 126; ++l) {
            cx126s3_sin_theta_1[l] = sin(big_angle_cx126s3[l / 3] * DEG_TO_RAD);
            cx126s3_cos_theta_1[l] = cos(big_angle_cx126s3[l / 3] * DEG_TO_RAD);

            cx126s3_sin_theta_2[l] = sin((l % 3) * (-0.14) * DEG_TO_RAD);
            cx126s3_cos_theta_2[l] = cos((l % 3) * (-0.14) * DEG_TO_RAD);
        }

        for (int k = 0; k < 16; ++k) {
            // 左边
            if (k / 4 % 2 == 0) {
                ch16x1_sin_theta_1[k] = sin((big_angle_ch16x1[k / 4] + prism_offset) * DEG_TO_RAD);
                ch16x1_cos_theta_1[k] = cos((big_angle_ch16x1[k / 4] + prism_offset) * DEG_TO_RAD);
            } else {
                ch16x1_sin_theta_1[k] = sin(big_angle_ch16x1[k / 4] * DEG_TO_RAD);
                ch16x1_cos_theta_1[k] = cos(big_angle_ch16x1[k / 4] * DEG_TO_RAD);
            }
            ch16x1_sin_theta_2[k] = sin((k % 4) * (-0.17) * DEG_TO_RAD);
            ch16x1_cos_theta_2[k] = cos((k % 4) * (-0.17) * DEG_TO_RAD);
        }

        for (int k1 = 0; k1 < 256; ++k1) {
            ch256_sin_theta_1[k1] = sin((-20.0 + floor(k1 / 4) * 0.625) * DEG_TO_RAD);
            ch256_cos_theta_1[k1] = cos((-20.0 + floor(k1 / 4) * 0.625) * DEG_TO_RAD);
            ch256_sin_theta_2[k1] = sin((k1 % 4) * 0.11 * DEG_TO_RAD);
            ch256_cos_theta_2[k1] = cos((k1 % 4) * 0.11 * DEG_TO_RAD);
        }

        if (!createRosIO()) {
            LS_ERROR << "Cannot create all ROS IO..." << LS_END;
            return false;
        }

        if (!getLidarEcho()) {
            LS_ERROR << "Cannot to obtain lidar echo mode..." << LS_END;
            return false;
        }

        coordinateConverters["CX1S3"]   = [this](Firing &data){ convertCoordinate_CX1S3(data); };
        coordinateConverters["CX6S3"]   = [this](Firing &data){ convertCoordinate_CX6S3(data); };
        coordinateConverters["CH16X1"]  = [this](Firing &data){ convertCoordinate_CH16X1(data); };
        coordinateConverters["CH64W"]   = [this](Firing &data){ convertCoordinate_CH64W(data); };
        coordinateConverters["CB64S1-A"]= [this](Firing &data){ convertCoordinate_CB64S1_A(data); };
        coordinateConverters["CX126S3"] = [this](Firing &data){ convertCoordinate_CX126S3(data); };
        coordinateConverters["CH128X1"] = [this](Firing &data){ convertCoordinate_128(data); };
        coordinateConverters["CH128S1"] = [this](Firing &data){ convertCoordinate_128(data); };
        coordinateConverters["CX128S2"] = [this](Firing &data){ convertCoordinate_CX128S2(data); };
        coordinateConverters["CH256"]   = [this](Firing &data){ convertCoordinate_CH256(data); };

        // 根据雷达类型绑定处理函数
        auto it = coordinateConverters.find(lidar_type);
        if (it != coordinateConverters.end()) {
            currentConverter = it->second;
        } else {
            throw std::invalid_argument("Unsupported lidar type. Please check the 'lidar_type' parameter in the launch file for correct options.");
        }

        point_cloud_xyzirt_->header.frame_id = frame_id;
        point_cloud_xyzirt_->height = 1;

        if (publish_laserscan) {
            scan_msg->angle_min = DEG2RAD(0);
            scan_msg->angle_max = DEG2RAD(180);
            scan_msg->range_min = min_range;
            scan_msg->range_max = max_range;
            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;

            point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
            if (lidar_type == "CH64W") { point_size *= 2; }
            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        }

        return true;
    }

    void LslidarChDriver::publishLaserScan() {
        if (!is_update_difop_packet) { return; }
        std::unique_lock<std::mutex> lock(pointcloud_lock);
        scan_msg_bak->header.frame_id = frame_id;
        scan_msg_bak->header.stamp = rclcpp::Time(point_cloud_timestamp * 1e9);
        laserscan_pub->publish(std::move(scan_msg_bak));
    }

    void LslidarChDriver::publishPointCloud() {
        if (!is_update_difop_packet) { return; }
        std::unique_lock<std::mutex> lock(pointcloud_lock);
        sensor_msgs::msg::PointCloud2 pc_msg;
        pcl::toROSMsg(*point_cloud_xyzirt_bak_, pc_msg);
        pc_msg.header.stamp = rclcpp::Time(point_cloud_timestamp * 1e9);
        pointcloud_pub->publish(pc_msg);
    }

    bool LslidarChDriver::polling() {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_ch_driver::msg::LslidarPacket::UniquePtr packet(new lslidar_ch_driver::msg::LslidarPacket());
        int rc = -1;
        // Since the lslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            // keep reading until full packet received
            rc = msop_input_->getPacket(packet);

            if (rc == 0) break;       // got a full packet?
            if (rc < 0) return false; // end of file reached?
        }

        if (use_time_service) {
            if ("CH128X1" == lidar_type || "CH64W" == lidar_type || "CB64S1-A" == lidar_type) {
                if (packet->data[1205] == 0x01) {
                    this->packetTimeStamp[4] = packet->data[1199];
                    this->packetTimeStamp[5] = packet->data[1198];
                    this->packetTimeStamp[6] = packet->data[1197];
                } else if (packet->data[1205] == 0x02) {
                    this->packetTimeStamp[4] = packet->data[1199];
                }

                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_sec  = this->packetTimeStamp[4];
                cur_time.tm_min  = this->packetTimeStamp[5];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_mon  = this->packetTimeStamp[8] - 1;
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
                packet_timestamp_s = timegm(&cur_time);
                
                if (time_service_mode == "GPS") {          //gps
                    packet_timestamp_ns = (packet->data[1203] +
                                          (packet->data[1202] << 8) +
                                          (packet->data[1201] << 16) +
                                          (packet->data[1200] << 24)) * 1e3; //ns
                } else {               //ptp
                    packet_timestamp_ns = packet->data[1203] +
                                         (packet->data[1202] << 8) +
                                         (packet->data[1201] << 16) +
                                         (packet->data[1200] << 24); //ns
                }
                packet_timestamp = packet_timestamp_s + packet_timestamp_ns * 1e-9;
            } else {
                if (packet->data[1200] == 0xff) {             //ptp
                    packet_timestamp_s = packet->data[1205] +
                                        (packet->data[1204] << 8) +
                                        (packet->data[1203] << 16) +
                                        (packet->data[1202] << 24);
                } else {
                    memset(&cur_time, 0, sizeof(cur_time));
                    cur_time.tm_sec  = packet->data[1205];
                    cur_time.tm_min  = packet->data[1204];
                    cur_time.tm_hour = packet->data[1203];
                    cur_time.tm_mday = packet->data[1202];
                    cur_time.tm_mon  = packet->data[1201] - 1;
                    cur_time.tm_year = packet->data[1200] + 2000 - 1900;
                    packet_timestamp_s = timegm(&cur_time);
                }
                packet_timestamp_ns = packet->data[1209] +
                                     (packet->data[1208] << 8) +
                                     (packet->data[1207] << 16) +
                                     (packet->data[1206] << 24); //ns
                packet_timestamp = packet_timestamp_s + packet_timestamp_ns * 1e-9;
            }
        } else {
            packet_timestamp = this->get_clock()->now().seconds();
        }

        packet_interval_time = packet_timestamp - last_packet_timestamp;
        
        packetProcess(packet);   //解析数据包

        last_packet_timestamp = packet_timestamp;
    
        return true;
    }

    void LslidarChDriver::packetProcessSingle(const lslidar_ch_driver::msg::LslidarPacket::UniquePtr &packet) {
        struct Firing lidardata{};
        bool packetType = false;
        point_interval_time = packet_interval_time * SINGLE_ECHO;

        for (size_t point_idx = 0, point_num = 0; point_idx < 1197; point_idx += 7, ++point_num) {
            if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) && (packet->data[point_idx + 2] == 0xbb) && 
                (packet->data[point_idx + 3] == 0xcc) && (packet->data[point_idx + 4] == 0xdd)) {
                packetType = true;
                point_cloud_timestamp = last_packet_timestamp + point_interval_time * point_num;
            } else {
                int point_azimuth = (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                if ((point_azimuth < scan_start_angle) || (point_azimuth > scan_end_angle)) continue;

                double point_distance = ((packet->data[point_idx + 3] << 16) + (packet->data[point_idx + 4] << 8) +
                                            packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                if ((point_distance < min_range) || (point_distance > max_range)) continue;
                
                memset(&lidardata, 0, sizeof(lidardata));
                lidardata.vertical_line = packet->data[point_idx];
                lidardata.azimuth = point_azimuth;
                lidardata.distance = point_distance;
                lidardata.intensity = packet->data[point_idx + 6];
                lidardata.time = last_packet_timestamp + point_interval_time * (point_num + 1) - point_cloud_timestamp;
                convertCoordinate(lidardata);
            }

            if (packetType) {
                {
                    std::unique_lock<std::mutex> lock(pointcloud_lock);
                    point_cloud_xyzirt_bak_ = std::move(point_cloud_xyzirt_);
                    scan_msg_bak = std::move(scan_msg);
                }
                threadPool_->enqueue(&LslidarChDriver::publishPointCloud, this);

                if (publish_laserscan) {
                    threadPool_->enqueue(&LslidarChDriver::publishLaserScan, this);
                }

                packetType = false;

                point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                point_cloud_xyzirt_->header.frame_id = frame_id;
                point_cloud_xyzirt_->height = 1;

                if (publish_laserscan) {
                    scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
                    scan_msg->angle_min = DEG2RAD(0);
                    scan_msg->angle_max = DEG2RAD(180);
                    scan_msg->range_min = min_range;
                    scan_msg->range_max = max_range;
                    scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
                    point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
                    if (lidar_type == "CH64W") { point_size *= 2; }
                    scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                    scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                }
            }
        }
    }

    void LslidarChDriver::packetProcessDouble(const lslidar_ch_driver::msg::LslidarPacket::UniquePtr &packet) {
        struct Firing lidardata{};
        bool packetType = false;
        point_interval_time = packet_interval_time * DOUBLE_ECHO;

        for (size_t point_idx = 0, point_num = 0; point_idx < 1199; point_idx += 11, ++point_num) {
            if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) &&
                (packet->data[point_idx + 2] == 0xbb) && (packet->data[point_idx + 3] == 0xcc)) {
                packetType = true;
                point_cloud_timestamp = last_packet_timestamp + point_interval_time * point_num;
            } else {
                int point_azimuth = (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                if ((point_azimuth < scan_start_angle) || (point_azimuth > scan_end_angle)) continue;

                double first_point_distance = ((packet->data[point_idx + 3] << 16) + (packet->data[point_idx + 4] << 8) +
                                                packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                if ((first_point_distance < min_range) || (first_point_distance > max_range)) continue;

                double second_point_distance = ((packet->data[point_idx + 7] << 16) + (packet->data[point_idx + 8] << 8) +
                                                packet->data[point_idx + 9]) * DISTANCE_RESOLUTION;
                if ((second_point_distance < min_range) || (second_point_distance > max_range)) continue;

                double point_time = last_packet_timestamp + point_interval_time * (point_num + 1) - point_cloud_timestamp;
                if (echo_num == 0) {
                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.vertical_line = packet->data[point_idx];
                    lidardata.azimuth = point_azimuth;
                    lidardata.distance = first_point_distance;
                    lidardata.intensity = packet->data[point_idx + 6];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);

                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.vertical_line = packet->data[point_idx];
                    lidardata.azimuth = point_azimuth;
                    lidardata.distance = second_point_distance;
                    lidardata.intensity = packet->data[point_idx + 10];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);
                } else if (echo_num == 1) {
                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.vertical_line = packet->data[point_idx];
                    lidardata.azimuth = point_azimuth;
                    lidardata.distance = first_point_distance;
                    lidardata.intensity = packet->data[point_idx + 6];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);
                } else if (echo_num == 2) {
                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.vertical_line = packet->data[point_idx];
                    lidardata.azimuth = point_azimuth;
                    lidardata.distance = second_point_distance;
                    lidardata.intensity = packet->data[point_idx + 10];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);
                }
            }

            if (packetType) {
                {
                    std::unique_lock<std::mutex> lock(pointcloud_lock);
                    point_cloud_xyzirt_bak_ = std::move(point_cloud_xyzirt_);
                    scan_msg_bak = std::move(scan_msg);
                }
                threadPool_->enqueue(&LslidarChDriver::publishPointCloud, this);

                if (publish_laserscan) {
                    threadPool_->enqueue(&LslidarChDriver::publishLaserScan, this);
                }

                packetType = false;

                point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                point_cloud_xyzirt_->header.frame_id = frame_id;
                point_cloud_xyzirt_->height = 1;

                if (publish_laserscan) {
                    scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
                    scan_msg->angle_min = DEG2RAD(0);
                    scan_msg->angle_max = DEG2RAD(180);
                    scan_msg->range_min = min_range;
                    scan_msg->range_max = max_range;
                    scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
                    point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
                    if (lidar_type == "CH64W") { point_size *= 2; }
                    scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                    scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                }
            }
        }
    }

    void LslidarChDriver::initTimeStamp(void) {
        for (int i = 0; i < 10; i++) {
            this->packetTimeStamp[i] = 0;
        }
    }

    void LslidarChDriver::initializeMaxChannelNumbers(void) {
        maxChannelNumbers["CX1S3"]    = 1;
        maxChannelNumbers["CX6S3"]    = 6;
        maxChannelNumbers["CH16X1"]   = 16;
        maxChannelNumbers["CH64W"]    = 64;
        maxChannelNumbers["CB64S1-A"] = 64;
        maxChannelNumbers["CX126S3"]  = 126;
        maxChannelNumbers["CH128X1"]  = 128;
        maxChannelNumbers["CH128S1"]  = 128;
        maxChannelNumbers["CX128S2"]  = 128;
        maxChannelNumbers["CH256"]    = 256;
    }


    void LslidarChDriver::difopPoll(void) {
        lslidar_ch_driver::msg::LslidarPacket::UniquePtr difop_packet(new lslidar_ch_driver::msg::LslidarPacket());

        // reading and publishing scans as fast as possible.
        while (rclcpp::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc == 0) {
                if (difop_packet->data[0] == 0xa5 && difop_packet->data[1] == 0xff && 
                    difop_packet->data[2] == 0x00 && difop_packet->data[3] == 0x5a) {
                    is_update_difop_packet = true;
                    
                    if (use_time_service){
                        if (difop_packet->data[44] == 0x00) {
                            time_service_mode = "GPS";
                        } else if (difop_packet->data[44] == 0x01) {
                            time_service_mode = "PTP";
                        }
                        RCLCPP_INFO_ONCE(this->get_logger(),"Lidar using %s time service.",time_service_mode.c_str());
                    }

                    if (gain_prism_angle) {
                        // 240 241   左边 增加角度
                        auto process_value = [&difop_packet](int index) {
                            int value = difop_packet->data[index] * 256 + difop_packet->data[index + 1];
                            if (value > 32767) {
                                value -= 65536;
                            }
                            return value * 0.01;
                        };

                        this->prism_offset   = process_value(240);
                        this->prism_angle[0] = process_value(242);
                        this->prism_angle[1] = process_value(244);
                        this->prism_angle[2] = process_value(246);
                        this->prism_angle[3] = process_value(248);

                        
                        if (lidar_type == "CH128X1" || lidar_type == "CH128S1" || lidar_type == "CX128S2") {
                            for (int i = 0; i < 128; i++) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 && fabs(this->prism_angle[3]) < 1e-6) {
                                    sin_theta_2[i] = sin((i % 4) * (-0.17) * M_PI / 180);
                                    cos_theta_2[i] = cos((i % 4) * (-0.17) * M_PI / 180);
                                } else {
                                    sin_theta_2[i] = sin(this->prism_angle[i % 4] * M_PI / 180);
                                    cos_theta_2[i] = cos(this->prism_angle[i % 4] * M_PI / 180);
                                }

                                // 左边
                                if (i / 4 % 2 == 0) {
                                    sin_theta_1[i] = sin((big_angle[i / 4] + prism_offset) * DEG_TO_RAD);
                                    cos_theta_1[i] = cos((big_angle[i / 4] + prism_offset) * DEG_TO_RAD);
                                } else {
                                    sin_theta_1[i] = sin(big_angle[i / 4] * DEG_TO_RAD);
                                    cos_theta_1[i] = cos(big_angle[i / 4] * DEG_TO_RAD);
                                }

                                if (lidar_type == "CH128S1" || lidar_type == "CX128S2") {
                                    // 左边
                                    if (i / 4 % 2 == 0) {
                                        sin_theta_1[i] = sin((big_angle_ch128s1[i / 4] + prism_offset) * DEG_TO_RAD);
                                        cos_theta_1[i] = cos((big_angle_ch128s1[i / 4] + prism_offset) * DEG_TO_RAD);
                                    } else {
                                        sin_theta_1[i] = sin(big_angle_ch128s1[i / 4] * DEG_TO_RAD);
                                        cos_theta_1[i] = cos(big_angle_ch128s1[i / 4] * DEG_TO_RAD);
                                    }
                                }
                            }
                        } else if (lidar_type == "CH256") {
                            for (int i = 0; i < 256; i++) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 && fabs(this->prism_angle[3]) < 1e-6) {
                                    ch256_sin_theta_2[i] = sin((i % 4) * 0.11 * DEG_TO_RAD);
                                    ch256_cos_theta_2[i] = cos((i % 4) * 0.11 * DEG_TO_RAD);
                                } else {
                                    ch256_sin_theta_2[i] = sin(this->prism_angle[i % 4] * DEG_TO_RAD);
                                    ch256_cos_theta_2[i] = cos(this->prism_angle[i % 4] * DEG_TO_RAD);
                                }
                                // 左边
                                if (i / 4 % 2 == 0) {
                                    ch256_sin_theta_1[i] = sin((-20.0 + floor(i / 4) * 0.625 + prism_offset) * DEG_TO_RAD);
                                    ch256_cos_theta_1[i] = cos((-20.0 + floor(i / 4) * 0.625 + prism_offset) * DEG_TO_RAD);
                                } else {
                                    ch256_sin_theta_1[i] = sin((-20.0 + floor(i / 4) * 0.625) * DEG_TO_RAD);
                                    ch256_cos_theta_1[i] = cos((-20.0 + floor(i / 4) * 0.625) * DEG_TO_RAD);
                                }
                            }
                        }else if (lidar_type == "CX126S3") {
                            for (int i = 0; i < 126; i++) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6) {
                                    cx126s3_sin_theta_2[i] = sin((i % 3) * (-0.14) * DEG_TO_RAD);
                                    cx126s3_cos_theta_2[i] = cos((i % 3) * (-0.14) * DEG_TO_RAD);
                                } else {
                                    cx126s3_sin_theta_2[i] = sin(this->prism_angle[i % 3] * DEG_TO_RAD);
                                    cx126s3_cos_theta_2[i] = cos(this->prism_angle[i % 3] * DEG_TO_RAD);
                                }

                                if (i / 3 % 2 == 0) { // 左边
                                    sin_theta_1[i] = sin((big_angle_cx126s3[i / 3] + prism_offset) * DEG_TO_RAD);
                                    cos_theta_1[i] = cos((big_angle_cx126s3[i / 3] + prism_offset) * DEG_TO_RAD);
                                } else {
                                    sin_theta_1[i] = sin(big_angle_cx126s3[i / 3] * DEG_TO_RAD);
                                    cos_theta_1[i] = cos(big_angle_cx126s3[i / 3] * DEG_TO_RAD);
                                }
                            }
                        } else if (lidar_type == "CX6S3") {
                            for (int i = 0; i < 6; i++) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6) {
                                    cx6s3_sin_theta_2[i] = sin((i % 3) * (-0.14) * DEG_TO_RAD);
                                    cx6s3_cos_theta_2[i] = cos((i % 3) * (-0.14) * DEG_TO_RAD);
                                } else {
                                    cx6s3_sin_theta_2[i] = sin(this->prism_angle[i % 3] * DEG_TO_RAD);
                                    cx6s3_cos_theta_2[i] = cos(this->prism_angle[i % 3] * DEG_TO_RAD);
                                }

                                // 左边
                                if (i / 3 % 2 == 0) {
                                    cx6s3_sin_theta_1[i] = sin((big_angle_cx6s3[i / 3] + prism_offset) * DEG_TO_RAD);
                                    cx6s3_cos_theta_1[i] = cos((big_angle_cx6s3[i / 3] + prism_offset) * DEG_TO_RAD);
                                } else {
                                    cx6s3_sin_theta_1[i] = sin(big_angle_cx6s3[i / 3] * DEG_TO_RAD);
                                    cx6s3_cos_theta_1[i] = cos(big_angle_cx6s3[i / 3] * DEG_TO_RAD);
                                }
                            }
                        } else if (lidar_type == "CH16X1") {
                            for (int k = 0; k < 16; ++k) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 && fabs(this->prism_angle[3]) < 1e-6) {
                                    ch16x1_sin_theta_2[k] = sin((k % 4) * (-0.17) * DEG_TO_RAD);
                                    ch16x1_cos_theta_2[k] = cos((k % 4) * (-0.17) * DEG_TO_RAD);
                                } else {
                                    ch16x1_sin_theta_2[k] = sin(this->prism_angle[k % 4] * DEG_TO_RAD);
                                    ch16x1_cos_theta_2[k] = cos(this->prism_angle[k % 4] * DEG_TO_RAD);
                                }
                                // 左边
                                if (k / 4 % 2 == 0) {
                                    ch16x1_sin_theta_1[k] = sin((big_angle_ch16x1[k / 4] + prism_offset) * DEG_TO_RAD);
                                    ch16x1_cos_theta_1[k] = cos((big_angle_ch16x1[k / 4] + prism_offset) * DEG_TO_RAD);
                                } else {
                                    ch16x1_sin_theta_1[k] = sin(big_angle_ch16x1[k / 4] * DEG_TO_RAD);
                                    ch16x1_cos_theta_1[k] = cos(big_angle_ch16x1[k / 4] * DEG_TO_RAD);
                                }
                            }
                        } else if (lidar_type == "CH64W") {
                            for (int m = 0; m < 128; ++m) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 && fabs(this->prism_angle[3]) < 1e-6) {
                                    if (m / 4 % 2 == 0) {//右边
                                        ch64w_sin_theta_1[m] = sin((-25 + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_cos_theta_1[m] = cos((-25 + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_sin_theta_2[m] = sin((m % 4) * 0.35 * DEG_TO_RAD);
                                        ch64w_cos_theta_2[m] = cos((m % 4) * 0.35 * DEG_TO_RAD);
                                    } else { //左边
                                        ch64w_sin_theta_1[m] = sin((-24 + prism_offset + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_cos_theta_1[m] = cos((-24 + prism_offset + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_sin_theta_2[m] = sin((m % 4) * 0.35 * DEG_TO_RAD);
                                        ch64w_cos_theta_2[m] = cos((m % 4) * 0.35 * DEG_TO_RAD);
                                    }
                                } else {
                                    //右边
                                    if (m / 4 % 2 == 0) {
                                        ch64w_sin_theta_1[m] = sin((-25 + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_cos_theta_1[m] = cos((-25 + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_sin_theta_2[m] = sin(prism_angle[m % 4] * DEG_TO_RAD);
                                        ch64w_cos_theta_2[m] = cos(prism_angle[m % 4] * DEG_TO_RAD);
                                    } else { //左边
                                        ch64w_sin_theta_1[m] = sin((-24 + prism_offset + floor(m / 8) * 2.5) * DEG_TO_RAD);                   
                                        ch64w_cos_theta_1[m] = cos((-24 + prism_offset + floor(m / 8) * 2.5) * DEG_TO_RAD);
                                        ch64w_sin_theta_2[m] = sin(prism_angle[m % 4] * DEG_TO_RAD);
                                        ch64w_cos_theta_2[m] = cos(prism_angle[m % 4] * DEG_TO_RAD);
                                    }
                                }
                            }
                        } else if (lidar_type == "CB64S1-A") {
                            for (int m = 0; m < 64; ++m) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 && fabs(this->prism_angle[3]) < 1e-6) {
                                    cb64s1_A_sin_theta_1[m] = sin((-25 + floor(m / 4) * 2.5) * DEG_TO_RAD);                                      
                                    cb64s1_A_cos_theta_1[m] = cos((-25 + floor(m / 4) * 2.5) * DEG_TO_RAD);
                                    cb64s1_A_sin_theta_2[m] = sin((m % 4) * 0.35 * DEG_TO_RAD);
                                    cb64s1_A_cos_theta_2[m] = cos((m % 4) * 0.35 * DEG_TO_RAD);
                                } else {
                                    cb64s1_A_sin_theta_1[m] = sin((-25 + floor(m / 4) * 2.5) * DEG_TO_RAD);
                                    cb64s1_A_cos_theta_1[m] = cos((-25 + floor(m / 4) * 2.5) * DEG_TO_RAD);
                                    cb64s1_A_sin_theta_2[m] = sin(prism_angle[m % 4] * DEG_TO_RAD);                                     
                                    cb64s1_A_cos_theta_2[m] = cos(prism_angle[m % 4] * DEG_TO_RAD);                         
                                }
                            }
                        }
                        gain_prism_angle = false;
                    }
                    if ("CH64W" == lidar_type || "CB64S1-A" == lidar_type || "CH128X1" == lidar_type) {
                        if (difop_packet->data[176] == 0x00) {
                            this->packetTimeStamp[7] = difop_packet->data[54];
                            this->packetTimeStamp[8] = difop_packet->data[53];
                            this->packetTimeStamp[9] = difop_packet->data[52];
                        } else if (difop_packet->data[176] == 0x01) {
                            this->packetTimeStamp[5] = difop_packet->data[56];
                            this->packetTimeStamp[6] = difop_packet->data[55];
                            this->packetTimeStamp[7] = difop_packet->data[54];
                            this->packetTimeStamp[8] = difop_packet->data[53];
                            this->packetTimeStamp[9] = difop_packet->data[52];
                        }
                    }
                }
            } else if (rc < 0) {
                return;
            }
        }
    }

    bool LslidarChDriver::getLidarEcho(void){
        lslidar_ch_driver::msg::LslidarPacket::UniquePtr pkt(new lslidar_ch_driver::msg::LslidarPacket());
            
        while (true) {
            // keep reading until full packet received
            int rc_ = msop_input_->getPacket(pkt);

            if (rc_ == 0) break;       // got a full packet?
            if (rc_ < 0) return false;; // end of file reached?
        }

        if (pkt->data[echo_byte] == 0x01) {
            packetProcess = std::bind(&LslidarChDriver::packetProcessSingle, this, std::placeholders::_1);
            LS_INFO << "Lidar echo mode: single echo" << LS_END;
        } else if (pkt->data[echo_byte] == 0x02) {
            packetProcess = std::bind(&LslidarChDriver::packetProcessDouble, this, std::placeholders::_1);
            LS_INFO << "Lidar echo mode: double echo" << LS_END;
        } else {
            return false;
        }
        std::cout << std::endl;
        return true;
    }

    //convert coordinate
    void LslidarChDriver::convertCoordinate_CX1S3(struct Firing &lidardata) {
        const float x = lidardata.distance * cos_list[lidardata.azimuth];
        const float y = lidardata.distance * sin_list[lidardata.azimuth];
        const float z = 0.0;
        
        //add point
        point_cloud_xyzirt_->points.emplace_back(x, y, z, lidardata.intensity, lidardata.vertical_line, lidardata.time);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                float horizontal_angle = lidardata.azimuth * 0.01f * DEG_TO_RAD;;
                uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                scan_msg->ranges[point_index] = lidardata.distance;
                scan_msg->intensities[point_index] = lidardata.intensity;
            }
        }
    }

    void LslidarChDriver::convertCoordinate_CX6S3(struct Firing &lidardata) {
        const double _R_ = cx6s3_cos_theta_2[lidardata.vertical_line] * cx6s3_cos_theta_1[lidardata.vertical_line] *
                           cos_list[int(lidardata.azimuth * 0.5)] -
                           cx6s3_sin_theta_2[lidardata.vertical_line] * cx6s3_sin_theta_1[lidardata.vertical_line];

        const double sin_theat = cx6s3_sin_theta_1[lidardata.vertical_line] + 2 * _R_ * cx6s3_sin_theta_2[lidardata.vertical_line];         
        const double cos_theat = sqrt(1 - pow(sin_theat, 2));

        const float x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
        const float y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
        const float z = lidardata.distance * sin_theat;
        
        //add point
        point_cloud_xyzirt_->points.emplace_back(x, y, z, lidardata.intensity, lidardata.vertical_line, lidardata.time);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                float horizontal_angle = lidardata.azimuth * 0.01f * DEG_TO_RAD;;
                uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                scan_msg->ranges[point_index] = lidardata.distance;
                scan_msg->intensities[point_index] = lidardata.intensity;
            }
        }
    }

    void LslidarChDriver::convertCoordinate_CH16X1(struct Firing &lidardata) {
        const double _R_ = ch16x1_cos_theta_2[lidardata.vertical_line] * ch16x1_cos_theta_1[lidardata.vertical_line] *
                           cos_list[int(lidardata.azimuth * 0.5)] -
                           ch16x1_sin_theta_2[lidardata.vertical_line] * ch16x1_sin_theta_1[lidardata.vertical_line];
        const double sin_theat = ch16x1_sin_theta_1[lidardata.vertical_line] + 2 * _R_ * ch16x1_sin_theta_2[lidardata.vertical_line];  
        const double cos_theat = sqrt(1 - pow(sin_theat, 2));

        const float x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
        const float y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
        const float z = lidardata.distance * sin_theat;
    
        //add point
        point_cloud_xyzirt_->points.emplace_back(x, y, z, lidardata.intensity, lidardata.vertical_line, lidardata.time);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                float horizontal_angle = lidardata.azimuth * 0.01f * DEG_TO_RAD;;
                uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                scan_msg->ranges[point_index] = lidardata.distance;
                scan_msg->intensities[point_index] = lidardata.intensity;
            }          
        }
    }

    void LslidarChDriver::convertCoordinate_CH64W(struct Firing &lidardata) {
        resetVariables_CH64W();

        int line_num = lidardata.vertical_line;
        if (line_num / 4 % 2 == 0) {
            cos_xita = cos_list[int(lidardata.azimuth * 0.5 + 2250)];
            sin_xita = sin_list[int(lidardata.azimuth * 0.5 + 2250)];
        } else {
            int angle_tmp = int(11250 - lidardata.azimuth * 0.5) < 0 ? int(11250 - lidardata.azimuth * 0.5) + 36000 : int(11250 - lidardata.azimuth * 0.5);
            cos_xita = cos_list[angle_tmp];
            sin_xita = sin_list[angle_tmp];
        }
        _R_ = ch64w_cos_theta_2[line_num] * ch64w_cos_theta_1[line_num] * cos_xita -
              ch64w_sin_theta_2[line_num] * ch64w_sin_theta_1[line_num];

        sin_theat = ch64w_sin_theta_1[line_num] + 2 * _R_ * ch64w_sin_theta_2[line_num];
        cos_theat = sqrt(1 - pow(sin_theat, 2));

        cos_H_xita = (2 * _R_ * ch64w_cos_theta_2[line_num] * cos_xita - ch64w_cos_theta_1[line_num]) / cos_theat;
        sin_H_xita = (2 * _R_ * ch64w_cos_theta_2[line_num] * sin_xita) / cos_theat;

        if (line_num / 4 % 2 == 0) {
            cos_xita_F = (cos_H_xita + sin_H_xita) * sqrt_0_5;
            if (cos_xita_F > 1.0) {
                cos_xita_F = 1.0;
            }
            double xita_hangle = acos(cos_xita_F) * RAD_TO_DEG;
            double xita_hangle_new = pow1 * pow(xita_hangle, 3) + pow2 * pow(xita_hangle, 2)                                       
                                     + 0.9885 * pow(xita_hangle, 1) + 0.5894;                 
            while (xita_hangle_new < 0.0) {
                xita_hangle_new += 360.0;
            }
            int xita_hangle_new_index = int(xita_hangle_new * 100) % 36000;
            cos_xita_F = cos_list[xita_hangle_new_index];
            sin_xita_F = sin_list[xita_hangle_new_index];
            add_distance = 0.017;
        } else {
            cos_xita_F = (cos_H_xita + sin_H_xita) * (-sqrt_0_5);
            if (cos_xita_F < -1.0) {
                cos_xita_F = -1.0;
            }
            double xita_hangle = acos(cos_xita_F) * RAD_TO_DEG;
            double xita_hangle_new = pow1 * pow(xita_hangle, 3) + pow3 * pow(xita_hangle, 2)
                                     + 0.9719 * pow(xita_hangle, 1) + 1.9003;                                      
            while (xita_hangle_new < 0.0) {
                xita_hangle_new += 360.0;
            }
            int xita_hangle_new_index = int(xita_hangle_new * 100) % 36000;
            cos_xita_F = cos_list[xita_hangle_new_index];
            sin_xita_F = sin_list[xita_hangle_new_index];
            add_distance = -0.017;
        }

        x = lidardata.distance * cos_theat * cos_xita_F + add_distance;
        y = lidardata.distance * cos_theat * sin_xita_F;
        z = lidardata.distance * sin_theat;
        
        //add point
        point_cloud_xyzirt_->points.emplace_back(x, y, z, lidardata.intensity, lidardata.vertical_line, lidardata.time);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num / 4 % 2 == 0) {
                channel_num1 = channel_num;
                channel_num2 = channel_num + 4;
            } else {
                channel_num1 = channel_num;
                channel_num2 = channel_num - 4;
            }

            if (channel_num1 == lidardata.vertical_line || channel_num2 == lidardata.vertical_line) {             
                float horizontal_angle = lidardata.azimuth * 0.01f * DEG_TO_RAD;
                uint point_idx = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                point_idx = (point_idx < point_size) ? point_idx : (point_idx % point_size);
                scan_msg->ranges[point_idx] = lidardata.distance;
                scan_msg->intensities[point_idx] = lidardata.intensity;
            }
        }
    }

    void LslidarChDriver::convertCoordinate_CB64S1_A(struct Firing &lidardata) {
        resetVariables_CB64S1_A();

        int line_num = lidardata.vertical_line;
        {
            cos_xita = cos_list[int(lidardata.azimuth * 0.5)];
            sin_xita = sin_list[int(lidardata.azimuth * 0.5)];
        }

        _R_ = cb64s1_A_cos_theta_2[line_num] * cb64s1_A_cos_theta_1[line_num] * cos_xita -
              cb64s1_A_sin_theta_2[line_num] * cb64s1_A_sin_theta_1[line_num];

        sin_theat = cb64s1_A_sin_theta_1[line_num] + 2 * _R_ * cb64s1_A_sin_theta_2[line_num];
        cos_theat = sqrt(1 - pow(sin_theat, 2));

        x = lidardata.distance * cos_theat * cos_list[int(lidardata.azimuth)];
        y = lidardata.distance * cos_theat * sin_list[int(lidardata.azimuth)];
        z = lidardata.distance * sin_theat;
        
        //add point
        point_cloud_xyzirt_->points.emplace_back(x, y, z, lidardata.intensity, lidardata.vertical_line, lidardata.time);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {          
            if (channel_num == lidardata.vertical_line) {
                float horizontal_angle = lidardata.azimuth * 0.01f * DEG_TO_RAD;;
                uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                scan_msg->ranges[point_index] = lidardata.distance;
                scan_msg->intensities[point_index] = lidardata.intensity;
            }          
        }
    }

    void LslidarChDriver::convertCoordinate_CX126S3(struct Firing &lidardata) {
        const double _R_ = cx126s3_cos_theta_2[lidardata.vertical_line] * cx126s3_cos_theta_1[lidardata.vertical_line] *
                           cos_list[int(lidardata.azimuth * 0.5)] -
                           cx126s3_sin_theta_2[lidardata.vertical_line] * cx126s3_sin_theta_1[lidardata.vertical_line];
        const double sin_theat = cx126s3_sin_theta_1[lidardata.vertical_line] + 2 * _R_ * cx126s3_sin_theta_2[lidardata.vertical_line];     
        const double cos_theat = sqrt(1 - pow(sin_theat, 2));

        const float x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
        const float y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
        const float z = lidardata.distance * sin_theat;
        
        //add point
        point_cloud_xyzirt_->points.emplace_back(x, y, z, lidardata.intensity, lidardata.vertical_line, lidardata.time);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {       
            if (channel_num == lidardata.vertical_line) {
                float horizontal_angle = lidardata.azimuth * 0.01f * DEG_TO_RAD;;
                uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                scan_msg->ranges[point_index] = lidardata.distance;
                scan_msg->intensities[point_index] = lidardata.intensity;
            }          
        }
    }

    void LslidarChDriver::convertCoordinate_128(struct Firing &lidardata) {
	    const double _R_ = cos_theta_2[lidardata.vertical_line] * cos_theta_1[lidardata.vertical_line] *
                           cos_list[int(lidardata.azimuth * 0.5)] -
                           sin_theta_2[lidardata.vertical_line] * sin_theta_1[lidardata.vertical_line];
	    const double sin_theat = sin_theta_1[lidardata.vertical_line] + 2 * _R_ * sin_theta_2[lidardata.vertical_line];
	    const double cos_theat = sqrt(1 - pow(sin_theat, 2));

	    const float x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
	    const float y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
	    const float z = lidardata.distance * sin_theat;
        
        point_cloud_xyzirt_->points.emplace_back(x, y, z, lidardata.intensity, lidardata.vertical_line, lidardata.time);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                float horizontal_angle = lidardata.azimuth * 0.01f * DEG_TO_RAD;;
                uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                scan_msg->ranges[point_index] = lidardata.distance;
                scan_msg->intensities[point_index] = lidardata.intensity;
            }
        }
    }

    void LslidarChDriver::convertCoordinate_CX128S2(struct Firing &lidardata) {
        int angle_h = 0;
        if (3000 <= lidardata.azimuth && lidardata.azimuth <= 6000)
        {
            angle_h = (0.9822 * lidardata.azimuth * 0.01 + 1.322) * 100;
        }
        else if (6000 < lidardata.azimuth && lidardata.azimuth <= 12000)
        {
            angle_h = (0.9932 * lidardata.azimuth * 0.01 + 0.655) * 100;
        }
        else if (12000 < lidardata.azimuth && lidardata.azimuth <= 15000)
        {
            angle_h = (0.9873 * lidardata.azimuth * 0.01 + 1.3720) * 100;
        }
        
        const double _R_ = cos_theta_2[lidardata.vertical_line] * cos_theta_1[lidardata.vertical_line] *
                           cos_list[int(angle_h * 0.5)] -
                           sin_theta_2[lidardata.vertical_line] * sin_theta_1[lidardata.vertical_line];
	    const double sin_theat = sin_theta_1[lidardata.vertical_line] + 2 * _R_ * sin_theta_2[lidardata.vertical_line];
	    const double cos_theat = sqrt(1 - pow(sin_theat, 2));

	    const float x = lidardata.distance * cos_theat * cos_list[angle_h];
	    const float y = lidardata.distance * cos_theat * sin_list[angle_h];
	    const float z = lidardata.distance * sin_theat;

        point_cloud_xyzirt_->points.emplace_back(x, y, z, lidardata.intensity, lidardata.vertical_line, lidardata.time);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                float horizontal_angle = lidardata.azimuth * 0.01f * DEG_TO_RAD;;
                uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                scan_msg->ranges[point_index] = lidardata.distance;
                scan_msg->intensities[point_index] = lidardata.intensity;
            }
        }
    }

    void LslidarChDriver::convertCoordinate_CH256(struct Firing &lidardata) {     
        const double _R_ = ch256_cos_theta_2[lidardata.vertical_line] * ch256_cos_theta_1[lidardata.vertical_line] *
                           cos_list[int(lidardata.azimuth * 0.5)] -
                           ch256_sin_theta_2[lidardata.vertical_line] * ch256_sin_theta_1[lidardata.vertical_line];
        const double sin_theat = ch256_sin_theta_1[lidardata.vertical_line] + 2 * _R_ * ch256_sin_theta_2[lidardata.vertical_line];               
        const double cos_theat = sqrt(1 - pow(sin_theat, 2));

        const float x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
        const float y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
        const float z = lidardata.distance * sin_theat;

        //add point
        point_cloud_xyzirt_->points.emplace_back(x, y, z, lidardata.intensity, lidardata.vertical_line, lidardata.time);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                float horizontal_angle = lidardata.azimuth * 0.01f * DEG_TO_RAD;;
                uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                scan_msg->ranges[point_index] = lidardata.distance;
                scan_msg->intensities[point_index] = lidardata.intensity;
            }
        }
    }
} // namespace lslidar_ch_driver

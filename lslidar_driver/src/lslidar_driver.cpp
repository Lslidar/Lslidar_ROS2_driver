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

#include <memory>

namespace lslidar_ch_driver {
    LslidarChDriver::LslidarChDriver() : LslidarChDriver(rclcpp::NodeOptions()) {
        return;
    }

    LslidarChDriver::LslidarChDriver(
            const rclcpp::NodeOptions &options) :
            rclcpp::Node("lslidar_node", options),
            socket_id(-1),
            point_cloud_xyzirt_(new pcl::PointCloud<VPoint>),
            point_cloud_xyzi_(new pcl::PointCloud<pcl::PointXYZI>),
            point_cloud_xyzirt_bak_(new pcl::PointCloud<VPoint>),
            point_cloud_xyzi_bak_(new pcl::PointCloud<pcl::PointXYZI>),
            scan_msg(new sensor_msgs::msg::LaserScan()),
            scan_msg_bak(new sensor_msgs::msg::LaserScan()),
            time_service_mode("gps"),
            gain_prism_angle(true) {
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
        if (nullptr == difop_thread_) {
            difop_thread_->join();
        }
        return;
    }

    bool LslidarChDriver::loadParameters() {
        this->declare_parameter<std::string>("pcap", "");
        this->declare_parameter<std::string>("device_ip", "192.168.1.200");
        this->declare_parameter<std::string>("lidar_type", "CH1W");
        this->declare_parameter<int>("msop_port", 2368);
        this->declare_parameter<int>("difop_port", 2369);
        this->declare_parameter<bool>("pcl_type", false);
        this->declare_parameter<bool>("add_multicast", false);
        this->declare_parameter<std::string>("group_ip", "234.2.3.2");
        this->declare_parameter<std::string>("frame_id", "laser_link");
        this->declare_parameter<double>("min_range", 0.3);
        this->declare_parameter<double>("max_range", 150.0);
        this->declare_parameter<double>("packet_rate", 1695.0);
        this->declare_parameter<int>("scan_start_angle", 0);
        this->declare_parameter<int>("scan_end_angle", 18000);
        this->declare_parameter<std::string>("topic_name", "lslidar_point_cloud");
        this->declare_parameter<double>("horizontal_angle_resolution", 0.2);
        this->declare_parameter<bool>("use_time_service", false);
        this->declare_parameter<bool>("publish_scan", false);
        this->declare_parameter<int>("echo_num", 0);

        this->get_parameter("pcap", dump_file);
        this->get_parameter("device_ip", lidar_ip_string);
        this->get_parameter("lidar_type", lidar_type);
        this->get_parameter("msop_port", msop_udp_port);
        this->get_parameter("difop_port", difop_udp_port);
        this->get_parameter("pcl_type", pcl_type);
        this->get_parameter("add_multicast", add_multicast);
        this->get_parameter("group_ip", group_ip_string);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("min_range", min_range);
        this->get_parameter("max_range", max_range);
        this->get_parameter("packet_rate", packet_rate);
        this->get_parameter("scan_start_angle", scan_start_angle);
        this->get_parameter("scan_end_angle", scan_end_angle);
        this->get_parameter("topic_name", pointcloud_topic);
        this->get_parameter("horizontal_angle_resolution", horizontal_angle_resolution);
        this->get_parameter("use_time_service", use_time_service);
        this->get_parameter("publish_scan", publish_laserscan);
        this->get_parameter("echo_num", echo_num);

        return true;
    }

    void LslidarChDriver::outputParameters() {
        LS_PARAM << "******** CH1W ROS2 driver version: 1.0.0 ********" << LS_END;
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
        
        LS_SOCKET << "Accepting packets from IP address: " << lidar_ip_string.c_str() << LS_END;
        if (add_multicast) LS_SOCKET << "Opening UDP socket: group address: " << group_ip_string.c_str() << LS_END;
        LS_SOCKET << "Opening UDP socket msop port: " << msop_udp_port << LS_END;
        LS_SOCKET << "Opening UDP socket difop port: " << difop_udp_port << LS_END;
        std::cout << std::endl;
    }

    bool LslidarChDriver::createRosIO() {
        // ROS diagnostics
        pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
        laserscan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        if (dump_file != "") {
            msop_input_.reset(new lslidar_ch_driver::InputPCAP(this, msop_udp_port, 1206, packet_rate, dump_file));
            difop_input_.reset(new lslidar_ch_driver::InputPCAP(this, difop_udp_port, 1206, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_ch_driver::InputSocket(this, msop_udp_port, 1206));
            difop_input_.reset(new lslidar_ch_driver::InputSocket(this, difop_udp_port, 1206));
        }

        difop_thread_ = std::make_shared<std::thread>([this]() { difopPoll(); });
        return true;
    }

    bool LslidarChDriver::initialize() {
        this->initTimeStamp();
        if (!loadParameters()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot load all required ROS parameters...");
            return false;
        }

        outputParameters();

        for (double &j : prism_angle) {
            j = 0.0f;
        }

        for (int m = 0; m < 8; ++m) {
            ch1w_sin_theta_1[m] = sin(0);
            ch1w_sin_theta_2[m] = sin(0);
            ch1w_cos_theta_1[m] = cos(0);
            ch1w_cos_theta_2[m] = cos(0);
        }

        if (!createRosIO()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot create all ROS IO...");
            return false;
        }
        point_cloud_xyzirt_->header.frame_id = frame_id;
        point_cloud_xyzirt_->height = 1;

        point_cloud_xyzi_->header.frame_id = frame_id;
        point_cloud_xyzi_->height = 1;
        if (publish_laserscan) {
            scan_msg->angle_min = DEG2RAD(0);
            scan_msg->angle_max = DEG2RAD(180);
            scan_msg->range_min = min_range;
            scan_msg->range_max = max_range;
            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
            point_size = 15820;
            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        }

        return true;
    }

    void LslidarChDriver::publishLaserScan() {
        std::unique_lock<std::mutex> lock(pointcloud_lock);
        if (!is_update_difop_packet) { return; }
        scan_msg_bak->header.frame_id = frame_id;
        scan_msg_bak->header.stamp = rclcpp::Time(point_cloud_timestamp * 1e9);
        laserscan_pub->publish(std::move(scan_msg_bak));
    }

    void LslidarChDriver::publishPointCloud() {
        if (!is_update_difop_packet) { return; }
        std::unique_lock<std::mutex> lock(pointcloud_lock);

        if (pcl_type) {
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud_xyzi_bak_, pc_msg);
            pc_msg.header.stamp = rclcpp::Time(point_cloud_timestamp * 1e9);
            pointcloud_pub->publish(pc_msg);
        } else {
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud_xyzirt_bak_, pc_msg);
            pc_msg.header.stamp = rclcpp::Time(point_cloud_timestamp * 1e9);
            pointcloud_pub->publish(pc_msg);
        }
        return;
    }


    int LslidarChDriver::convertCoordinate(struct Firing &lidardata) {
        double x = 0.0, y = 0.0, z = 0.0;
        double sin_theat = 0.0;
        double cos_theat = 0.0;
        double add_distance = 0.0;
        double _R_ = 0.0;

        double cos_xita;
        double sin_xita;
        double cos_H_xita;
        double sin_H_xita;
        double cos_xita_F;
        double sin_xita_F;

        int line_num = lidardata.vertical_line;
        if (line_num / 4 % 2 == 0) {
            cos_xita = cos_list[int(lidardata.azimuth * 0.5 + 2250)];
            sin_xita = sin_list[int(lidardata.azimuth * 0.5 + 2250)];
        } else {
            int angle_tmp =
                    int(11250 - lidardata.azimuth * 0.5) < 0 ? int(11250 - lidardata.azimuth * 0.5) + 36000 : int(
                            11250 - lidardata.azimuth * 0.5);
            cos_xita = cos_list[angle_tmp];
            sin_xita = sin_list[angle_tmp];
        }
        _R_ = ch1w_cos_theta_2[line_num] * ch1w_cos_theta_1[line_num] * cos_xita -
                ch1w_sin_theta_2[line_num] * ch1w_sin_theta_1[line_num];

        sin_theat = ch1w_sin_theta_1[line_num] + 2 * _R_ * ch1w_sin_theta_2[line_num];
        cos_theat = sqrt(1 - pow(sin_theat, 2));

        cos_H_xita = (2 * _R_ * ch1w_cos_theta_2[line_num] * cos_xita - ch1w_cos_theta_1[line_num]) / cos_theat;
        sin_H_xita = (2 * _R_ * ch1w_cos_theta_2[line_num] * sin_xita) / cos_theat;

        if (line_num / 4 % 2 == 0) {
            cos_xita_F = (cos_H_xita + sin_H_xita) * sqrt_0_5;
            if (cos_xita_F > 1.0) {
                cos_xita_F = 1.0;
            }
            double xita_hangle = acos(cos_xita_F) * RAD_TO_DEG;
            double xita_hangle_new = pow1 * pow(xita_hangle, 3)
                                        + pow2 * pow(xita_hangle, 2)
                                        + 0.9885 * pow(xita_hangle, 1)
                                        + 0.5894;
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
            double xita_hangle_new = pow1 * pow(xita_hangle, 3)
                                        + pow3 * pow(xita_hangle, 2)
                                        + 0.9719 * pow(xita_hangle, 1)
                                        + 1.9003;
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
        if (pcl_type) {
            pcl::PointXYZI point_xyzi;
            point_xyzi.x = x;
            point_xyzi.y = y;
            point_xyzi.z = z;
            point_xyzi.intensity = lidardata.intensity;
            point_cloud_xyzi_->points.push_back(point_xyzi);
            ++point_cloud_xyzi_->width;
        } else {
            //pcl::PointXYZI point;
            VPoint point;
            point.time = lidardata.time;
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = lidardata.intensity;
            point.ring = lidardata.vertical_line;
            point_cloud_xyzirt_->points.push_back(point);
            ++point_cloud_xyzirt_->width;
        }
        // laserscan
        if (publish_laserscan) {
            float horizontal_angle = lidardata.azimuth * 0.01f * DEG_TO_RAD;
            uint point_idx = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
            point_idx = (point_idx < point_size) ? point_idx : (point_idx % point_size);
            scan_msg->ranges[point_idx] = lidardata.distance;
            scan_msg->intensities[point_idx] = lidardata.intensity;
        }

        return 0;
    }

    bool LslidarChDriver::polling() {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet(new lslidar_msgs::msg::LslidarPacket());
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
            if (packet->data[1205] == 0x01) {
                this->packetTimeStamp[4] = packet->data[1199];
                this->packetTimeStamp[5] = packet->data[1198];
                this->packetTimeStamp[6] = packet->data[1197];
            } else if (packet->data[1205] == 0x02) {
                this->packetTimeStamp[4] = packet->data[1199];
            }

            memset(&cur_time, 0, sizeof(cur_time));
            cur_time.tm_sec = this->packetTimeStamp[4];
            cur_time.tm_min = this->packetTimeStamp[5];
            cur_time.tm_hour = this->packetTimeStamp[6];
            cur_time.tm_mday = this->packetTimeStamp[7];
            cur_time.tm_mon = this->packetTimeStamp[8] - 1;
            cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
            packet_timestamp_s = timegm(&cur_time);
            if (time_service_mode == "gps") {          //gps
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
            packet_timestamp = this->get_clock()->now().seconds();
        }

        struct Firing lidardata{};
        packet_interval_time = packet_timestamp - last_packet_timestamp;
        
        bool packetType = false;

        if (packet->data[1205] == 0x01) {
            point_interval_time = packet_interval_time * SINGLE_ECHO;
            for (size_t point_idx = 0, point_num = 0; point_idx < 1197; point_idx += 7, ++point_num) {
                if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) && (packet->data[point_idx + 2] == 0xbb) && 
                    (packet->data[point_idx + 3] == 0xcc) && (packet->data[point_idx + 4] == 0xdd)) {
                    point_cloud_timestamp = last_packet_timestamp + point_interval_time * point_num;
                    if (point_cloud_timestamp < 0.0) point_cloud_timestamp = 0.0;
                    packetType = true;
                }

                if (packet->data[point_idx] < 128) {
                    int point_azimuth = (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                    if ((point_azimuth < scan_start_angle) || (point_azimuth > scan_end_angle)) continue;

                    double point_distance = ((packet->data[point_idx + 3] << 16) + (packet->data[point_idx + 4] << 8) +
                                              packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                    if ((point_distance < min_range) || (point_distance > max_range)) continue;
                    
                    double point_time = last_packet_timestamp + point_interval_time * (point_num + 1) - point_cloud_timestamp;

                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.vertical_line = packet->data[point_idx];
                    lidardata.azimuth = point_azimuth;
                    lidardata.distance = point_distance;
                    lidardata.intensity = packet->data[point_idx + 6];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);
                }
                if (packetType) {
                    //("---------------onesweep--------------------------\n");
                    {
                        std::unique_lock<std::mutex> lock(pointcloud_lock);
                        point_cloud_xyzirt_bak_ = point_cloud_xyzirt_;
                        point_cloud_xyzi_bak_ = point_cloud_xyzi_;
                        scan_msg_bak = std::move(scan_msg);
                    }
                    std::thread ppc_thread(&LslidarChDriver::publishPointCloud, this);
                    ppc_thread.detach();

                    if (publish_laserscan) {
                        std::thread pls_thread(&LslidarChDriver::publishLaserScan, this);
                        pls_thread.detach();
                    }

                    packetType = false;
                    point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                    point_cloud_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>);
                    point_cloud_xyzirt_->header.frame_id = frame_id;
                    point_cloud_xyzirt_->height = 1;

                    point_cloud_xyzi_->header.frame_id = frame_id;
                    point_cloud_xyzi_->height = 1;
                    if (publish_laserscan) {
                        scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
                        //scan_msg.reset(new sensor_msgs::msg::LaserScan);
                        scan_msg->angle_min = DEG2RAD(0);
                        scan_msg->angle_max = DEG2RAD(180);
                        scan_msg->range_min = min_range;
                        scan_msg->range_max = max_range;
                        scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
                        point_size = 15820;
                        scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                        scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());

                    }
                }
            }
        } else if (packet->data[1205] == 0x02) {
            RCLCPP_INFO_ONCE(this->get_logger(),
                                "lidar is double echo model,and the selected echo is: %d [0 mean double echo; 1 mean first echo; 2 mean second echo]",
                                echo_num);
            point_interval_time = packet_interval_time * DOUBLE_ECHO;
            for (size_t point_idx = 0, point_num = 0; point_idx < 1199; point_idx += 11, ++point_num) {
                if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) && (packet->data[point_idx + 2] == 0xbb) && 
                    (packet->data[point_idx + 3] == 0xcc) && (packet->data[point_idx + 4] == 0xdd)) {
                    packetType = true;
                    point_cloud_timestamp = last_packet_timestamp + point_interval_time * point_num;
                    if (point_cloud_timestamp < 0.0) point_cloud_timestamp = 0.0;
                }
                if (packet->data[point_idx] < 128) {
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
                    //("---------------onesweep--------------------------\n");
                    {
                        std::unique_lock<std::mutex> lock(pointcloud_lock);
                        point_cloud_xyzirt_bak_ = point_cloud_xyzirt_;
                        point_cloud_xyzi_bak_ = point_cloud_xyzi_;
                        scan_msg_bak = std::move(scan_msg);
                    }
                    std::thread ppc_thread(&LslidarChDriver::publishPointCloud, this);
                    ppc_thread.detach();

                    if (publish_laserscan) {
                        std::thread pls_thread(&LslidarChDriver::publishLaserScan, this);
                        pls_thread.detach();
                    }
                    packetType = false;
                    point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                    point_cloud_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>);
                    point_cloud_xyzirt_->header.frame_id = frame_id;
                    point_cloud_xyzirt_->height = 1;

                    point_cloud_xyzi_->header.frame_id = frame_id;
                    point_cloud_xyzi_->height = 1;
                    if (publish_laserscan) {
                        scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
                        scan_msg->angle_min = DEG2RAD(0);
                        scan_msg->angle_max = DEG2RAD(180);
                        scan_msg->range_min = min_range;
                        scan_msg->range_max = max_range;
                        scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
                        point_size = 15820;
                        scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                        scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                    }
                }
            }
        }

        last_packet_timestamp = packet_timestamp;

        return true;
    }

    void LslidarChDriver::initTimeStamp(void) {

        for (int i = 0; i < 10; i++) {
            this->packetTimeStamp[i] = 0;
        }
    }

    void LslidarChDriver::difopPoll(void) {
        lslidar_msgs::msg::LslidarPacket::UniquePtr difop_packet(new lslidar_msgs::msg::LslidarPacket());

        // reading and publishing scans as fast as possible.
        while (rclcpp::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc == 0) {
                // getFPGA_GPSTimeStamp(difop_packet);
                if (difop_packet->data[0] == 0xa5 && difop_packet->data[1] == 0xff && difop_packet->data[2] == 0x00 &&
                    difop_packet->data[3] == 0x5a) {
                    is_update_difop_packet = true;
                    if (difop_packet->data[44] == 0x00) {
                        //gps授时
                        time_service_mode = "gps";
                    } else if (difop_packet->data[44] == 0x01) {
                        //ptp授时
                        time_service_mode = "gptp";
                    }

                    if (gain_prism_angle) {
                        // 240 241   左边 增加角度
                        int prism_offset_difop = difop_packet->data[240] * 256 + difop_packet->data[241];

                        prism_offset_difop =
                                prism_offset_difop > 32767 ? prism_offset_difop - 65536 : prism_offset_difop;
                        this->prism_offset = prism_offset_difop * 0.01;
                        //ROS_INFO("prism=%f",prism_offset);

                        int angle0 = difop_packet->data[242] * 256 + difop_packet->data[243];
                        angle0 = angle0 > 32767 ? (angle0 - 65536) : angle0;
                        this->prism_angle[0] = angle0 * 0.01;
                        //ROS_INFO("0aa=%f",prism_angle[0]);

                        int angle1 = difop_packet->data[244] * 256 + difop_packet->data[245];
                        angle1 = angle1 > 32767 ? (angle1 - 65536) : angle1;
                        this->prism_angle[1] = angle1 * 0.01;
                        //ROS_INFO("1aa=%f",prism_angle[1]);

                        int angle2 = difop_packet->data[246] * 256 + difop_packet->data[247];
                        angle2 = angle2 > 32767 ? (angle2 - 65536) : angle2;
                        this->prism_angle[2] = angle2 * 0.01;
                        //ROS_INFO("2aa=%f",prism_angle[2]);

                        int angle3 = difop_packet->data[248] * 256 + difop_packet->data[249];
                        angle3 = angle3 > 32767 ? (angle3 - 65536) : angle3;
                        this->prism_angle[3] = angle3 * 0.01;
                        //ROS_INFO("3aa=%f",prism_angle[3]);


                        for (int m = 0; m < 8; ++m) {
                            if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                fabs(this->prism_angle[2]) < 1e-6 && fabs(this->prism_angle[3]) < 1e-6) {
                                for (int m = 0; m < 8; ++m) {
                                    ch1w_sin_theta_1[m] = sin(0);
                                    ch1w_sin_theta_2[m] = sin(0);
                                    ch1w_cos_theta_1[m] = cos(0);
                                    ch1w_cos_theta_2[m] = cos(0);
                                }
                            } else {
                                //右边
                                if (m / 4 % 2 == 0) {
                                    ch1w_sin_theta_1[m] = sin(0);
                                    ch1w_sin_theta_2[m] = sin(prism_angle[m % 4] * DEG_TO_RAD);
                                    ch1w_cos_theta_1[m] = cos(0);
                                    ch1w_cos_theta_2[m] = cos(prism_angle[m % 4] * DEG_TO_RAD);
                                } else { //左边
                                    ch1w_sin_theta_1[m] = sin((0 + prism_offset) * DEG_TO_RAD);
                                    ch1w_sin_theta_2[m] = sin(prism_angle[m % 4] * DEG_TO_RAD);
                                    ch1w_cos_theta_1[m] = cos((0 + prism_offset) * DEG_TO_RAD);   
                                    ch1w_cos_theta_2[m] = cos(prism_angle[m % 4] * DEG_TO_RAD);
                                }
                            }
                        }

                        gain_prism_angle = false;
                    }

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

            } else if (rc < 0) {
                return;
            }
        }
    }

} // namespace lslidar_driver

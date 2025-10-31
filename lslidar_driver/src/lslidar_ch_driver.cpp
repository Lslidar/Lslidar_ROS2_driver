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

#include <lslidar_driver/lslidar_ch_driver.hpp>

namespace lslidar_driver {
    LslidarChDriver::LslidarChDriver(rclcpp::Node::SharedPtr node): LslidarDriver(node),
                                                                    point_cloud_xyzirt_(new pcl::PointCloud<VPoint>()),
                                                                    point_cloud_xyzirt_bak_(new pcl::PointCloud<VPoint>()),
                                                                    scan_msg(new sensor_msgs::msg::LaserScan()),
                                                                    scan_msg_bak(new sensor_msgs::msg::LaserScan()),
                                                                    prism_offset(0.0),
                                                                    gain_prism_angle(true),
                                                                    last_packet_timestamp(0.0) {
        services_ = std::make_shared<LslidarChServices>();
    }

    bool LslidarChDriver::loadParameters() {
        node_->declare_parameter<std::string>("pcap", "");
        node_->declare_parameter<double>("packet_rate", 6737.0);
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

        node_->declare_parameter<int>("scan_start_angle", 0);
        node_->declare_parameter<int>("scan_end_angle", 18000);
        node_->declare_parameter<double>("horizontal_angle_resolution", 0.15);
        node_->declare_parameter<bool>("publish_scan", false);
        node_->declare_parameter<int>("scan_num", 0);
        node_->declare_parameter<int>("echo_mode", 0);
        node_->declare_parameter<std::string>("lidar_model", "CH128X1");
        
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

        node_->get_parameter("scan_start_angle", scan_start_angle);
        node_->get_parameter("scan_end_angle", scan_end_angle);
        node_->get_parameter("horizontal_angle_resolution", horizontal_angle_resolution);
        node_->get_parameter("publish_scan", publish_laserscan);
        node_->get_parameter("scan_num", channel_num);
        node_->get_parameter("echo_mode", echo_mode);
        node_->get_parameter("lidar_model", lidar_model);

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

        if ((!tTransform_main.empty()) && (!tTransform_imu.empty()))
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

    void LslidarChDriver::outputParameters() {
        LS_PARAM << "lidar model: " << lidar_model << LS_END;
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
        LS_PARAM << "scan start angle: " << scan_start_angle << LS_END;
        LS_PARAM << "scan end angle: " << scan_end_angle << LS_END;
        LS_PARAM << "echo_mode: " << echo_mode << LS_END;
        LS_PARAM << "publish scan: " << publish_laserscan << LS_END;
        LS_PARAM << "scan num: " << channel_num << LS_END;

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

    bool LslidarChDriver::createRosIO() {
        pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10); 
        if (publish_laserscan) laserscan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        lidar_info_pub_ = node_->create_publisher<lslidar_msgs::msg::LslidarInformation>("lslidar_device_info", 1);
        time_pub_ = node_->create_publisher<std_msgs::msg::Float64>("time_topic", 10);

        network_config_service_ = node_->create_service<lslidar_msgs::srv::IpAndPort>("network_setup",
            [this](std::shared_ptr<lslidar_msgs::srv::IpAndPort::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::IpAndPort::Response> res) {
                std::dynamic_pointer_cast<LslidarChServices>(services_)->setIpAndPort(req, res);
            });

        motor_speed_service_ = node_->create_service<lslidar_msgs::srv::MotorSpeed>("motor_speed",
            [this](std::shared_ptr<lslidar_msgs::srv::MotorSpeed::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::MotorSpeed::Response> res) {
                std::dynamic_pointer_cast<LslidarChServices>(services_)->setMotorSpeed(req, res);
            });
        
        time_mode_service_ = node_->create_service<lslidar_msgs::srv::TimeMode>("time_mode",
            [this](std::shared_ptr<lslidar_msgs::srv::TimeMode::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::TimeMode::Response> res) {
                std::dynamic_pointer_cast<LslidarChServices>(services_)->setTimeMode(req, res);
            });

        if (dump_file != "") {
            if (lidar_model == "CH128S1" || lidar_model == "CH16X1" || lidar_model == "CX128S2" ||
                lidar_model == "CX126S3" || lidar_model == "CX6S3" || lidar_model == "CH256" || lidar_model == "CX1S3") {
                msop_input_.reset(new lslidar_driver::InputPCAP(node_, msop_udp_port, 1212, packet_rate, dump_file));
                difop_input_.reset(new lslidar_driver::InputPCAP(node_, difop_udp_port, 1206, 1, dump_file));
            } else {
                msop_input_.reset(new lslidar_driver::InputPCAP(node_, msop_udp_port, 1206, packet_rate, dump_file));
                difop_input_.reset(new lslidar_driver::InputPCAP(node_, difop_udp_port, 1206, 1, dump_file));
                echo_byte = 1205;
            }
        } else {
            if (lidar_model == "CH128S1" || lidar_model == "CH16X1" || lidar_model == "CX128S2" ||
                lidar_model == "CX126S3" || lidar_model == "CX6S3" || lidar_model == "CH256" || lidar_model == "CX1S3") {
                msop_input_.reset(new lslidar_driver::InputSocket(node_, msop_udp_port, 1212));
                difop_input_.reset(new lslidar_driver::InputSocket(node_, difop_udp_port, 1206));
            } else {
                msop_input_.reset(new lslidar_driver::InputSocket(node_, msop_udp_port, 1206));
                difop_input_.reset(new lslidar_driver::InputSocket(node_, difop_udp_port, 1206));
                echo_byte = 1205;
            }
        }

        thread_pool_->enqueue([this]() { difopPoll(); });
                
        return true;
    }

    void LslidarChDriver::initTimeStamp(void) {
        for (int i = 0; i < 10; i++) {
            this->packetTimeStamp[i] = 0;
        }

        point_time_offset = use_first_point_time ? 2 : 0;
        relative_time_offset = use_absolute_time ? 0 : 1;
    }

    void LslidarChDriver::initAngleConfig() {
        for (double &j : prism_angle) {
            j = 0.0f;
        }

        for (int j = 0; j < 36000; ++j) {
            sin_list[j] = sin(j * 0.01 * DEG_TO_RAD);
            cos_list[j] = cos(j * 0.01 * DEG_TO_RAD);
        }

        // CH32A
        for (int i = 0; i < 8; ++i) {
            sin_scan_laser_altitude[i] = sin(scan_laser_altitude_1[i] * DEG_TO_RAD);
        }

        //ch256
        for (int k1 = 0; k1 < 256; ++k1) {
            ch256_sin_theta_1[k1] = sin((-20.0 + floor(k1 / 4) * 0.625) * DEG_TO_RAD);
            ch256_cos_theta_1[k1] = cos((-20.0 + floor(k1 / 4) * 0.625) * DEG_TO_RAD);
            ch256_sin_theta_2[k1] = sin((k1 % 4) * 0.11 * DEG_TO_RAD);
            ch256_cos_theta_2[k1] = cos((k1 % 4) * 0.11 * DEG_TO_RAD);
        }

        if(lidar_model == "CH1W")
        {
            // CH1W
            for (int m = 0; m < 128; ++m) {
                ch64w_sin_theta_1[m] = sin(0);
                ch64w_sin_theta_2[m] = sin(0);
                ch64w_cos_theta_1[m] = cos(0);
                ch64w_cos_theta_2[m] = cos(0);
            }
        }
        else
        {
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
        }



        //CB64S1_A
        for (int m = 0; m < 64; ++m) {
            //右边
            cb64s1_A_sin_theta_1[m] = sin((-25 + floor(m / 4) * 2.5) * DEG_TO_RAD);
            cb64s1_A_sin_theta_2[m] = sin((m % 4) * 0.35 * DEG_TO_RAD);
            cb64s1_A_cos_theta_1[m] = cos((-25 + floor(m / 4) * 2.5) * DEG_TO_RAD);
            cb64s1_A_cos_theta_2[m] = cos((m % 4) * 0.35 * DEG_TO_RAD);
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

            if (lidar_model == "CH128S1" || lidar_model == "CX128S2") {
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

        for (int i1 = 0; i1 < 6; ++i1) {
            cx6s3_sin_theta_1[i1] = sin(big_angle_cx6s3[i1 / 3] * DEG_TO_RAD);
            cx6s3_cos_theta_1[i1] = cos(big_angle_cx6s3[i1 / 3] * DEG_TO_RAD);

            cx6s3_sin_theta_2[i1] = sin((i1 % 3) * (-0.14) * DEG_TO_RAD);
            cx6s3_cos_theta_2[i1] = cos((i1 % 3) * (-0.14) * DEG_TO_RAD);
        }

        for (int k = 0; k < 16; ++k) {
            // 左边  prism_offset = 0.0
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
    }

    bool LslidarChDriver::initialize() {
        if (!loadParameters()) {
            LS_ERROR << "Cannot load all required ROS parameters..." << LS_END;
            return false;
        }

        this->outputParameters();

        if (!createRosIO()) {
            LS_ERROR << "Cannot create all ROS IO..." << LS_END;
            return false;
        }

        if (!getLidarEcho()) {
            LS_ERROR << "Cannot to obtain lidar echo mode..." << LS_END;
            return false;
        }

        this->initAngleConfig();

        this->initTimeStamp();

        if (is_pretreatment) {
            pointcloud_transform_.setTransform(x_offset, y_offset, z_offset, roll, pitch, yaw);
        }

        coordinateConverters["CX1S3"] = [this](Firing &data){ convertCoordinate_cx1s3(data); };
        coordinateConverters["CX6S3"] = [this](Firing &data){ convertCoordinate_cx6s3(data); };
        coordinateConverters["CH16X1"] = [this](Firing &data){ convertCoordinate_ch16x1(data); };
        coordinateConverters["CH32A"] = [this](Firing &data){ convertCoordinate_ch32a(data); };
        coordinateConverters["CH1W"] = [this](Firing &data){ convertCoordinate_ch64w(data); };
        coordinateConverters["CH64W"] = [this](Firing &data){ convertCoordinate_ch64w(data); };
        coordinateConverters["CB64S1_A"] = [this](Firing &data){ convertCoordinate_cb64s1_a(data); };
        coordinateConverters["CX126S3"] = [this](Firing &data){ convertCoordinate_cx126s3(data); };
        coordinateConverters["CH128X1"] = [this](Firing &data){ convertCoordinate_ch128(data); };
        coordinateConverters["CH128S1"] = [this](Firing &data){ convertCoordinate_ch128(data); };
        coordinateConverters["CX128S2"] = [this](Firing &data){ convertCoordinate_cx128s2(data); };
        coordinateConverters["CH256"] = [this](Firing &data){ convertCoordinate_ch256(data); };

        // 根据雷达类型绑定处理函数
        auto it = coordinateConverters.find(lidar_model);
        if (it != coordinateConverters.end()) {
            currentConverter = it->second;
        } else {
            throw std::invalid_argument("Unsupported lidar type. Please check the 'lidar_model' parameter in the launch file for correct options.");
        }

        point_cloud_xyzirt_->header.frame_id = frame_id;
        point_cloud_xyzirt_->height = 1;

        if (publish_laserscan) {
            scan_msg->angle_min = -M_PI;
            scan_msg->angle_max = M_PI;
            scan_msg->range_min = min_range;
            scan_msg->range_max = max_range;
            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
            point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
            if (lidar_model == "CH1W" || lidar_model == "CH64W" || lidar_model == "CB64S1_A") { point_size *= 2; }
            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::infinity());
            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        }

        return true;
    }

    void LslidarChDriver::publishLaserScan() {
        if (!is_get_difop_.load()) { return; }
        scan_msg_bak->header.frame_id = frame_id;
        scan_msg_bak->header.stamp = rclcpp::Time(point_cloud_time * 1000000000LL);
        laserscan_pub_->publish(std::move(scan_msg_bak));
    }

    void LslidarChDriver::publishPointcloud() {
        if (!is_get_difop_.load()) { return; }
        std::unique_lock<std::mutex> lock(pointcloud_lock);

        if (point_cloud_xyzirt_bak_->points.size() < 100) return;
        if (is_pretreatment) pointcloud_transform_.applyTransform(*point_cloud_xyzirt_bak_);   
        if (is_MatrixTransformation) pointcloud_transform_.applyTransform_2(*point_cloud_xyzirt_bak_, MatrixTransform_result);
        
        auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();     
        pcl::toROSMsg(*point_cloud_xyzirt_bak_, *pc_msg);
        pc_msg->header.stamp = rclcpp::Time(point_cloud_time * 1000000000LL);
        pointcloud_pub_->publish(*pc_msg);

        auto time_msg = std::make_shared<std_msgs::msg::Float64>();
        time_msg->data = point_cloud_time;
        time_pub_->publish(*time_msg);
    }

    void LslidarChDriver::difopPoll(void) {
        lslidar_msgs::msg::LslidarPacket::UniquePtr difop_packet(new lslidar_msgs::msg::LslidarPacket());   
        // reading and publishing scans as fast as possible.
        while (rclcpp::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc >= 1206) {
                if (difop_packet->data[0] == 0xA5 && difop_packet->data[1] == 0xFF &&
                    difop_packet->data[2] == 0x00 && difop_packet->data[3] == 0x5A) {
                    is_get_difop_.store(true);
                    if (difop_packet->data[44] == 0x00) {
                        //gps授时
                        time_service_mode = "gps";
                    } else if (difop_packet->data[44] == 0x01) {
                        //ptp授时
                        time_service_mode = "gptp";
                    }

                    // 服务配置雷达传递设备包
                    services_->getDifopPacket(difop_packet);

                    Information device = device_info_->getDeviceInfo(difop_packet);

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

                        // LS_INFO << "  prism_offset: " << this->prism_offset   << LS_END;
                        // LS_INFO << "prism_angle[0]: " << this->prism_angle[0] << LS_END;
                        // LS_INFO << "prism_angle[1]: " << this->prism_angle[1] << LS_END;
                        // LS_INFO << "prism_angle[2]: " << this->prism_angle[2] << LS_END;
                        // LS_INFO << "prism_angle[3]: " << this->prism_angle[3] << LS_END;
 
                        if (lidar_model == "CH128X1" || lidar_model == "CH128S1" || lidar_model == "CX128S2") {
                            for (int i = 0; i < 128; i++) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 && fabs(this->prism_angle[3]) < 1e-6) {                             
                                    sin_theta_2[i] = sin((i % 4) * (-0.17) * DEG_TO_RAD);
                                    cos_theta_2[i] = cos((i % 4) * (-0.17) * DEG_TO_RAD);
                                } else {
                                    sin_theta_2[i] = sin(this->prism_angle[i % 4] * DEG_TO_RAD);
                                    cos_theta_2[i] = cos(this->prism_angle[i % 4] * DEG_TO_RAD);
                                }

                                // 左边
                                if (i / 4 % 2 == 0) {
                                    sin_theta_1[i] = sin((big_angle[i / 4] + prism_offset) * DEG_TO_RAD);
                                    cos_theta_1[i] = cos((big_angle[i / 4] + prism_offset) * DEG_TO_RAD);

                                } else {
                                    sin_theta_1[i] = sin(big_angle[i / 4] * DEG_TO_RAD);
                                    cos_theta_1[i] = cos(big_angle[i / 4] * DEG_TO_RAD);
                                }

                                if (lidar_model == "CH128S1" || lidar_model == "CX128S2") {
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
                        } else if (lidar_model == "CH256") {
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
                        } else if (lidar_model == "CX126S3") {
                            for (int i = 0; i < 126; i++) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6) {
                                    cx126s3_sin_theta_2[i] = sin((i % 3) * (-0.14) * DEG_TO_RAD);
                                    cx126s3_cos_theta_2[i] = cos((i % 3) * (-0.14) * DEG_TO_RAD);
                                } else {
                                    cx126s3_sin_theta_2[i] = sin(this->prism_angle[i % 3] * DEG_TO_RAD);
                                    cx126s3_cos_theta_2[i] = cos(this->prism_angle[i % 3] * DEG_TO_RAD);
                                }

                                // 左边
                                if (i / 3 % 2 == 0) {
                                    cx126s3_sin_theta_1[i] = sin((big_angle_cx126s3[i / 3] + prism_offset) * DEG_TO_RAD);
                                    cx126s3_cos_theta_1[i] = cos((big_angle_cx126s3[i / 3] + prism_offset) * DEG_TO_RAD);
                                } else {
                                    cx126s3_sin_theta_1[i] = sin(big_angle_cx126s3[i / 3] * DEG_TO_RAD);
                                    cx126s3_cos_theta_1[i] = cos(big_angle_cx126s3[i / 3] * DEG_TO_RAD);
                                }
                            }
                        } else if (lidar_model == "CX6S3") {
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
                        } else if (lidar_model == "CH16X1") {
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
                        } else if (lidar_model == "CH1W") {
                            for (int m = 0; m < 8; ++m) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 && fabs(this->prism_angle[3]) < 1e-6) {     
                                    //右边
                                    if (m / 4 % 2 == 0) {

                                    } else { //左边
                                        ch64w_sin_theta_1[m] = sin((0 + prism_offset) * DEG_TO_RAD);
                                        ch64w_cos_theta_1[m] = cos((0 + prism_offset) * DEG_TO_RAD);                                               
                                    }
                                } else {
                                    //右边
                                    if (m / 4 % 2 == 0) {
                                        ch64w_sin_theta_2[m] = sin(prism_angle[m % 4] * DEG_TO_RAD);
                                        ch64w_cos_theta_2[m] = cos(prism_angle[m % 4] * DEG_TO_RAD);
                                    } else { //左边
                                        ch64w_sin_theta_1[m] = sin(prism_offset * DEG_TO_RAD);                                        
                                        ch64w_cos_theta_1[m] = cos(prism_offset * DEG_TO_RAD);
                                        ch64w_sin_theta_2[m] = sin(prism_angle[m % 4] * DEG_TO_RAD);                                               
                                        ch64w_cos_theta_2[m] = cos(prism_angle[m % 4] * DEG_TO_RAD);
                                    }
                                }
                            }
                        } else if (lidar_model == "CH64W") {                 
                            for (int m = 0; m < 128; ++m) {
                                if (fabs(this->prism_angle[0]) < 1e-6 && fabs(this->prism_angle[1]) < 1e-6 &&
                                    fabs(this->prism_angle[2]) < 1e-6 && fabs(this->prism_angle[3]) < 1e-6) {     
                                    //右边
                                    if (m / 4 % 2 == 0) {
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
                        } else if (lidar_model == "CB64S1_A") {
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

                    if ("CH32A" == lidar_model) {
                        this->packetTimeStamp[7] = difop_packet->data[38];
                        this->packetTimeStamp[8] = difop_packet->data[37];
                        this->packetTimeStamp[9] = difop_packet->data[36];
                    }

                    if ("CH128X1" == lidar_model || "CH1W" == lidar_model || "CH64W" == lidar_model || "CB64S1_A" == lidar_model) {
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

    bool LslidarChDriver::poll() {
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet_(new lslidar_msgs::msg::LslidarPacket());

        while (rclcpp::ok()) {
            // keep reading until full packet received
            int rc = msop_input_->getPacket(packet_);
            if (rc >= 1206) break;       // got a full packet?
            if (rc == 0) continue;    // No full packet, retry
            if (rc < 0) return false; // System-level error, terminate,end of file reached?
        }

        if (use_time_service) {
            if ("CH128X1" == lidar_model || "CH1W" == lidar_model || "CH64W" == lidar_model || "CB64S1_A" == lidar_model || "CH32A" == lidar_model) {
                if (packet_->data[1205] == 0x01) {
                    this->packetTimeStamp[4] = packet_->data[1199];
                    this->packetTimeStamp[5] = packet_->data[1198];
                    this->packetTimeStamp[6] = packet_->data[1197];
                } else if (packet_->data[1205] == 0x02) {
                    this->packetTimeStamp[4] = packet_->data[1199];
                }

                //struct tm cur_time{};
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_sec  = this->packetTimeStamp[4];
                cur_time.tm_min  = this->packetTimeStamp[5];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_mon  = this->packetTimeStamp[8] - 1;
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;

                packet_timestamp_s = timegm(&cur_time);
                if (time_service_mode == "gps") {
                    packet_timestamp_ns = (packet_->data[1203] + (packet_->data[1202] << 8) +
                                          (packet_->data[1201] << 16) + (packet_->data[1200] << 24)) * 1e3; //ns
                } else {
                    packet_timestamp_ns = packet_->data[1203] + (packet_->data[1202] << 8) +
                                         (packet_->data[1201] << 16) +(packet_->data[1200] << 24); //ns
                }
                packet_timestamp = packet_timestamp_s + packet_timestamp_ns * 1e-9;
            } else {
                if (packet_->data[1200] == 0xff) {
                    packet_timestamp_s = packet_->data[1205] + (packet_->data[1204] << 8) +
                                        (packet_->data[1203] << 16) + (packet_->data[1202] << 24);
                } else {
                    memset(&cur_time, 0, sizeof(cur_time));
                    cur_time.tm_sec  = packet_->data[1205];
                    cur_time.tm_min  = packet_->data[1204];
                    cur_time.tm_hour = packet_->data[1203];
                    cur_time.tm_mday = packet_->data[1202];
                    cur_time.tm_mon  = packet_->data[1201] - 1;
                    cur_time.tm_year = packet_->data[1200] + 2000 - 1900;
                    packet_timestamp_s = timegm(&cur_time);
                }
                packet_timestamp_ns = packet_->data[1209] + (packet_->data[1208] << 8) +
                                     (packet_->data[1207] << 16) + (packet_->data[1206] << 24); //ns
                packet_timestamp = packet_timestamp_s + packet_timestamp_ns * 1e-9;
            }
        } else {
            packet_timestamp = node_->get_clock()->now().seconds();
        }

        packet_interval_time = packet_timestamp - last_packet_timestamp;

        packetProcess(packet_);

        last_packet_timestamp = packet_timestamp;

        return true;
    }

    void LslidarChDriver::packetProcessSingle(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) {
        Firing lidardata;
        packetType = false;
        point_interval_time = packet_interval_time * CH_SINGLE_ECHO;

        for (size_t point_idx = 0, point_num = 0; point_idx < 1197; point_idx += 7, ++point_num) {
            if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) &&
                (packet->data[point_idx + 2] == 0xbb) && (packet->data[point_idx + 3] == 0xcc)) {
                point_cloud_timestamp = last_packet_timestamp + point_interval_time * (point_num + point_time_offset);
                point_cloud_time = use_first_point_time ? last_point_cloud_time : point_cloud_timestamp;
                last_point_cloud_time = point_cloud_timestamp;
                packetType = true;
            } else {
                int point_azimuth = (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                if ((point_azimuth < scan_start_angle) || (point_azimuth > scan_end_angle)) continue;

                double point_distance = ((packet->data[point_idx + 3] << 16) + (packet->data[point_idx + 4] << 8) +
                                          packet->data[point_idx + 5]) * CH_DISTANCE_RESOLUTION;
                if ((point_distance < min_range) || (point_distance > max_range)) continue;

                memset(&lidardata, 0, sizeof(lidardata));
                lidardata.vertical_line = packet->data[point_idx];
                lidardata.azimuth = point_azimuth;
                lidardata.distance = point_distance;
                lidardata.intensity = packet->data[point_idx + 6];
                lidardata.time = last_packet_timestamp + (point_num + 1) * point_interval_time - point_cloud_timestamp * relative_time_offset;

                convertCoordinate(lidardata);
            }

            if (packetType) {
                //("---------------onesweep--------------------------\n");
                {
                    std::unique_lock<std::mutex> lock(pointcloud_lock);
                    point_cloud_xyzirt_bak_ = std::move(point_cloud_xyzirt_);
                    scan_msg_bak = std::move(scan_msg);
                }

                thread_pool_->enqueue([this]() { publishPointcloud(); });

                if (publish_laserscan) {
                    thread_pool_->enqueue([this]() { publishLaserScan(); });
                }
                
                packetType = false;
                point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>());
                point_cloud_xyzirt_->header.frame_id = frame_id;
                point_cloud_xyzirt_->height = 1;

                if (publish_laserscan) {
                    scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
                    scan_msg->angle_min = -M_PI;
                    scan_msg->angle_max = M_PI;
                    scan_msg->range_min = min_range;
                    scan_msg->range_max = max_range;
                    scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
                    point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
                    if (lidar_model == "CH1W" || "CH64W" == lidar_model || lidar_model == "CB64S1_A") { point_size *= 2; }
                    scan_msg->ranges.assign(point_size, std::numeric_limits<float>::infinity());
                    scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                }
            }
        }
    }

    void LslidarChDriver::packetProcessDouble(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) {
        Firing lidardata;     
        packetType = false;
        point_interval_time = packet_interval_time * CH_DOUBLE_ECHO;

        for (size_t point_idx = 0, point_num = 0; point_idx < 1199; point_idx += 11, ++point_num) {
            if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) &&
                (packet->data[point_idx + 2] == 0xbb) && (packet->data[point_idx + 3] == 0xcc)) {
                point_cloud_timestamp = last_packet_timestamp + point_interval_time * (point_num + point_time_offset);
                point_cloud_time = use_first_point_time ? last_point_cloud_time : point_cloud_timestamp;
                last_point_cloud_time = point_cloud_timestamp;
                packetType = true;
            } else {
                double point_distance = ((packet->data[point_idx + 3] << 16) + (packet->data[point_idx + 4] << 8) +
                                          packet->data[point_idx + 5]) * CH_DISTANCE_RESOLUTION;
                if ((point_distance < min_range) || (point_distance > max_range)) continue;
                // Compute the time of the point
                point_time = last_packet_timestamp + (point_num + 1) * point_interval_time - point_cloud_timestamp * relative_time_offset;
                if (echo_mode == 0) {
                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.azimuth = (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];                             
                    if ((lidardata.azimuth < scan_start_angle) || (lidardata.azimuth > scan_end_angle)) continue;
                    lidardata.distance = point_distance;
                    
                    lidardata.vertical_line = packet->data[point_idx];
                    lidardata.intensity = packet->data[point_idx + 6];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);
                                                      
                    lidardata.distance = ((packet->data[point_idx + 7] << 16) + (packet->data[point_idx + 8] << 8) +
                                           packet->data[point_idx + 9]) * CH_DISTANCE_RESOLUTION;
                    if ((lidardata.distance < min_range) || (lidardata.distance > max_range)) continue;
                    lidardata.intensity = packet->data[point_idx + 10];

                    convertCoordinate(lidardata);
                } else if (echo_mode == 1) {
                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.azimuth = (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                    if ((lidardata.azimuth < scan_start_angle) || (lidardata.azimuth > scan_end_angle)) continue;                                 
                    lidardata.distance = ((packet->data[point_idx + 3] << 16) + (packet->data[point_idx + 4] << 8) +
                                           packet->data[point_idx + 5]) * CH_DISTANCE_RESOLUTION;
                    if ((lidardata.distance < min_range) || (lidardata.distance > max_range)) continue;
                    lidardata.vertical_line = packet->data[point_idx];
                    lidardata.intensity = packet->data[point_idx + 6];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);
                } else if (echo_mode == 2) {
                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.azimuth = (packet->data[point_idx + 1] << 8) + packet->data[point_idx + 2];
                    if ((lidardata.azimuth < scan_start_angle) || (lidardata.azimuth > scan_end_angle)) continue;
                    lidardata.distance = ((packet->data[point_idx + 7] << 16) + (packet->data[point_idx + 8] << 8) +
                                           packet->data[point_idx + 9]) * CH_DISTANCE_RESOLUTION;
                    if ((lidardata.distance < min_range) || (lidardata.distance > max_range)) continue;
                    lidardata.vertical_line = packet->data[point_idx];
                    lidardata.intensity = packet->data[point_idx + 10];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);
                }
            }
            if (packetType) {
                //("---------------onesweep--------------------------\n");
                {
                    std::unique_lock<std::mutex> lock(pointcloud_lock);
                    point_cloud_xyzirt_bak_ = std::move(point_cloud_xyzirt_);
                    scan_msg_bak = std::move(scan_msg);
                }

                thread_pool_->enqueue([this]() { publishPointcloud(); });

                if (publish_laserscan) {
                    thread_pool_->enqueue([this]() { publishLaserScan(); });
                }

                packetType = false;
                point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>());
                point_cloud_xyzirt_->header.frame_id = frame_id;
                point_cloud_xyzirt_->height = 1;

                if (publish_laserscan) {
                    scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
                    scan_msg->angle_min = -M_PI;
                    scan_msg->angle_max = M_PI;
                    scan_msg->range_min = min_range;
                    scan_msg->range_max = max_range;
                    scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
                    point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
                    if (lidar_model == "CH1W" || lidar_model == "CH64W" || lidar_model == "CB64S1_A") { point_size *= 2; }
                    scan_msg->ranges.assign(point_size, std::numeric_limits<float>::infinity());
                    scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
                }
            }
        }
    }

    bool LslidarChDriver::getLidarEcho(void){
        lslidar_msgs::msg::LslidarPacket::UniquePtr pkt(new lslidar_msgs::msg::LslidarPacket());
        while (rclcpp::ok()) {
            // keep reading until full packet received
            int rc_ = msop_input_->getPacket(pkt);

            if (rc_ >= 1206) break;       // got a full packet?
            if (rc_ == 0) continue;
            if (rc_ < 0) return false; // end of file reached?
        }

        if(pkt->data[echo_byte] == 1) {
            packetProcess = std::bind(&LslidarChDriver::packetProcessSingle, this, std::placeholders::_1);
            LS_INFO << "Lidar echo mode: single echo" << LS_END;
        } else if(pkt->data[echo_byte] == 2){
            packetProcess = std::bind(&LslidarChDriver::packetProcessDouble, this, std::placeholders::_1);
            LS_INFO << "Lidar echo mode: double echo" << LS_END;
        } else {
            LS_WARN << "Wrong echo type!" << LS_END;
            return false;
        }

        return true;
    };

    void LslidarChDriver::updateLaserscan(float x, float y, float intensity) {
        float x_coord = x;
        float y_coord = y;

        double range = hypot(x_coord, y_coord);
        double angle = atan2(y_coord, x_coord);

        int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
        if (range < scan_msg->ranges[index] && index < static_cast<int>(scan_msg->ranges.size())) {
            scan_msg->ranges[index] = range;
            scan_msg->intensities[index] = intensity;
        }
    }

//convert coordinate
    void LslidarChDriver::convertCoordinate_cx1s3(struct Firing &lidardata) {
        const float x = lidardata.distance * cos_list[lidardata.azimuth];
        const float y = lidardata.distance * sin_list[lidardata.azimuth];
        const float z = 0.0;
        
        //add point
        VPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.vertical_line;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                updateLaserscan(x, y, lidardata.intensity);
            }
        }
    }

    void LslidarChDriver::convertCoordinate_cx6s3(struct Firing &lidardata) {
        const double _R_ = cx6s3_cos_theta_2[lidardata.vertical_line] * cx6s3_cos_theta_1[lidardata.vertical_line] *
                           cos_list[int(lidardata.azimuth * 0.5)] -
                           cx6s3_sin_theta_2[lidardata.vertical_line] * cx6s3_sin_theta_1[lidardata.vertical_line];

        const double sin_theat = cx6s3_sin_theta_1[lidardata.vertical_line] + 2 * _R_ * cx6s3_sin_theta_2[lidardata.vertical_line];         
        const double cos_theat = sqrt(1 - pow(sin_theat, 2));

        const float x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
        const float y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
        const float z = lidardata.distance * sin_theat;
        
        //add point
        VPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.vertical_line;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                updateLaserscan(x, y, lidardata.intensity);
            }
        }
    }

    void LslidarChDriver::convertCoordinate_ch16x1(struct Firing &lidardata) {
        const double _R_ = ch16x1_cos_theta_2[lidardata.vertical_line] * ch16x1_cos_theta_1[lidardata.vertical_line] *
                           cos_list[int(lidardata.azimuth * 0.5)] -
                           ch16x1_sin_theta_2[lidardata.vertical_line] * ch16x1_sin_theta_1[lidardata.vertical_line];
        const double sin_theat = ch16x1_sin_theta_1[lidardata.vertical_line] + 2 * _R_ * ch16x1_sin_theta_2[lidardata.vertical_line];  
        const double cos_theat = sqrt(1 - pow(sin_theat, 2));

        const float x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
        const float y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
        const float z = lidardata.distance * sin_theat;
    
        //add point
        VPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.vertical_line;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                updateLaserscan(x, y, lidardata.intensity);
            }
        }
    }

    void LslidarChDriver::convertCoordinate_ch32a(struct Firing &lidardata) {
        const double z_sin_altitude = sin_scan_laser_altitude[lidardata.vertical_line / 4] +
                                      2 * cos_list[int(lidardata.azimuth * 0.5)] *
                                      sin_scan_mirror_altitude[lidardata.vertical_line % 4];
        const double z_cos_altitude = sqrt(1 - z_sin_altitude * z_sin_altitude);

        const float x = lidardata.distance * z_cos_altitude * cos_list[lidardata.azimuth];
        const float y = lidardata.distance * z_cos_altitude * sin_list[lidardata.azimuth];
        const float z = lidardata.distance * z_sin_altitude;

        //add point
        VPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.vertical_line;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                updateLaserscan(x, y, lidardata.intensity);
            }
        }
    }

    void LslidarChDriver::convertCoordinate_ch64w(struct Firing &lidardata) {
        resetVariables_ch64w();

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
        VPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.vertical_line;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                updateLaserscan(x, y, lidardata.intensity);
            }
        }
    }

    void LslidarChDriver::convertCoordinate_cb64s1_a(struct Firing &lidardata) {
        resetVariables_cb64s1_a();

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
        VPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.vertical_line;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                updateLaserscan(x, y, lidardata.intensity);
            }
        }
    }


    void LslidarChDriver::convertCoordinate_cx126s3(struct Firing &lidardata) {
        const double _R_ = cx126s3_cos_theta_2[lidardata.vertical_line] * cx126s3_cos_theta_1[lidardata.vertical_line] *
                           cos_list[int(lidardata.azimuth * 0.5)] -
                           cx126s3_sin_theta_2[lidardata.vertical_line] * cx126s3_sin_theta_1[lidardata.vertical_line];
        const double sin_theat = cx126s3_sin_theta_1[lidardata.vertical_line] + 2 * _R_ * cx126s3_sin_theta_2[lidardata.vertical_line];     
        const double cos_theat = sqrt(1 - pow(sin_theat, 2));

        const float x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
        const float y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
        const float z = lidardata.distance * sin_theat;
        
        //add point
        VPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.vertical_line;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;
        
        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                updateLaserscan(x, y, lidardata.intensity);
            }
        }
    }

    void LslidarChDriver::convertCoordinate_ch128(struct Firing &lidardata) {
	    const double _R_ = cos_theta_2[lidardata.vertical_line] * cos_theta_1[lidardata.vertical_line] *
                           cos_list[int(lidardata.azimuth * 0.5)] -
                           sin_theta_2[lidardata.vertical_line] * sin_theta_1[lidardata.vertical_line];
	    const double sin_theat = sin_theta_1[lidardata.vertical_line] + 2 * _R_ * sin_theta_2[lidardata.vertical_line];
	    const double cos_theat = sqrt(1 - pow(sin_theat, 2));

	    const float x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
	    const float y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
	    const float z = lidardata.distance * sin_theat;
        
        VPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.vertical_line;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                updateLaserscan(x, y, lidardata.intensity);
            }
        }
    }

    void LslidarChDriver::convertCoordinate_cx128s2(struct Firing &lidardata) {
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

        VPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.vertical_line;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                updateLaserscan(x, y, lidardata.intensity);
            }
        }
    }

    void LslidarChDriver::convertCoordinate_ch256(struct Firing &lidardata) {     
        const double _R_ = ch256_cos_theta_2[lidardata.vertical_line] * ch256_cos_theta_1[lidardata.vertical_line] *
                           cos_list[int(lidardata.azimuth * 0.5)] -
                           ch256_sin_theta_2[lidardata.vertical_line] * ch256_sin_theta_1[lidardata.vertical_line];
        const double sin_theat = ch256_sin_theta_1[lidardata.vertical_line] + 2 * _R_ * ch256_sin_theta_2[lidardata.vertical_line];               
        const double cos_theat = sqrt(1 - pow(sin_theat, 2));

        const float x = lidardata.distance * cos_theat * cos_list[lidardata.azimuth];
        const float y = lidardata.distance * cos_theat * sin_list[lidardata.azimuth];
        const float z = lidardata.distance * sin_theat;

        //add point
        VPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.vertical_line;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        // laserscan
        if (publish_laserscan) {
            if (channel_num == lidardata.vertical_line) {
                updateLaserscan(x, y, lidardata.intensity);
            }
        }
    }
} // namespace lslidar_driver

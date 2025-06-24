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

#include <lslidar_driver/lslidar_ls_driver.hpp>
#include <algorithm>
#include <memory>

namespace lslidar_driver {
    LslidarLsDriver::LslidarLsDriver(rclcpp::Node::SharedPtr node) : LslidarDriver(node),
                                                                     packet_end_time(0.0),
                                                                     current_packet_time(0.0),
                                                                     last_packet_time(0.0),
                                                                     return_mode(1),
                                                                     g_fAngleAcc_V(0.01),
                                                                     packet_loss(false),
                                                                     is_add_frame_(false),
                                                                     get_ms06_param(true),
                                                                     last_packet_number_(-1), 
                                                                     total_packet_loss_(0),
                                                                     frame_count(0),
                                                                     point_cloud_xyzirt_(new pcl::PointCloud<VPoint>()),
                                                                     point_cloud_xyzirt_bak_(new pcl::PointCloud<VPoint>()),
                                                                     point_cloud_xyzirt_pub_(new pcl::PointCloud<VPoint>()) {
        services_ = std::make_shared<LslidarLsServices>();
    }

    bool LslidarLsDriver::loadParameters() {
        node_->declare_parameter<std::string>("pcap", "");
        node_->declare_parameter<double>("packet_rate", 15000.0);
        node_->declare_parameter<std::string>("frame_id", "laser_link");
        node_->declare_parameter<bool>("add_multicast", false);
        node_->declare_parameter<std::string>("group_ip", "234.2.3.2");
        node_->declare_parameter<bool>("use_time_service", false);
        node_->declare_parameter<std::string>("device_ip", "192.168.1.200");
        node_->declare_parameter<int>("msop_port", (int) MSOP_DATA_PORT_NUMBER);
        node_->declare_parameter<int>("difop_port", (int) DIFOP_DATA_PORT_NUMBER);
        node_->declare_parameter<std::string>("pointcloud_topic", "lslidar_point_cloud");
        node_->declare_parameter<double>("min_range", 0.3);
        node_->declare_parameter<double>("max_range", 150.0);
        node_->declare_parameter<bool>("is_pretreatment", false);
        node_->declare_parameter<double>("x_offset", 0.0);
        node_->declare_parameter<double>("y_offset", 0.0);
        node_->declare_parameter<double>("z_offset", 0.0);
        node_->declare_parameter<double>("roll", 0.0);
        node_->declare_parameter<double>("pitch", 0.0);
        node_->declare_parameter<double>("yaw", 0.0);

        node_->declare_parameter<int>("scan_start_angle", -6000);
        node_->declare_parameter<int>("scan_end_angle", 6000);
        node_->declare_parameter<bool>("packet_loss", false);

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
        node_->get_parameter("packet_loss", packet_loss);

        return true;
    }

    void LslidarLsDriver::outputParameters() {
        LS_PARAM << "pcap file: " << dump_file << LS_END;
        LS_PARAM << "packet rate: " << packet_rate << LS_END;
        LS_PARAM << "add multicast: " << std::boolalpha << add_multicast << LS_END;
        LS_PARAM << "use time service: " << std::boolalpha << use_time_service << LS_END;
        LS_PARAM << "pointcloud topic: " << pointcloud_topic << LS_END;
        LS_PARAM << "frame id: " << frame_id << LS_END;
        LS_PARAM << "min range: " << min_range << LS_END;
        LS_PARAM << "max range: "<< max_range << LS_END;
        LS_PARAM << "scan start angle: " << scan_start_angle << LS_END;
        LS_PARAM << "scan end angle: " << scan_end_angle << LS_END;
        LS_PARAM << "packet loss check: " << std::boolalpha << packet_loss << LS_END;

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

    bool LslidarLsDriver::createRosIO() {
        pointcloud_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10); 
        fault_code_pub = node_->create_publisher<std_msgs::msg::String>("lslidar_fault_code", 1);
        lidar_info_pub = node_->create_publisher<lslidar_msgs::msg::LslidarInformation>("lslidar_device_info", 1);
        if (packet_loss) packet_loss_pub = node_->create_publisher<std_msgs::msg::Int64>("packet_loss", 10);
        time_pub = node_->create_publisher<std_msgs::msg::Float64>("time_topic", 10);

        angle_distortion_correction_service = node_->create_service<lslidar_msgs::srv::AngleDistortionCorrection>("angle_distortion_correction",
            [this](std::shared_ptr<lslidar_msgs::srv::AngleDistortionCorrection::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::AngleDistortionCorrection::Response> res) {
                std::dynamic_pointer_cast<LslidarLsServices>(services_)->setAngleDistortionCorrection(req, res);
            });

        network_config_service = node_->create_service<lslidar_msgs::srv::IpAndPort>("network_setup",
            [this](std::shared_ptr<lslidar_msgs::srv::IpAndPort::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::IpAndPort::Response> res) {
                std::dynamic_pointer_cast<LslidarLsServices>(services_)->setIpAndPort(req, res);
            });

        time_mode_service = node_->create_service<lslidar_msgs::srv::TimeMode>("time_mode",
            [this](std::shared_ptr<lslidar_msgs::srv::TimeMode::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::TimeMode::Response> res) {
                std::dynamic_pointer_cast<LslidarLsServices>(services_)->setTimeMode(req, res);
            });

        frame_rate_service = node_->create_service<lslidar_msgs::srv::FrameRate>("frame_rate",
            [this](std::shared_ptr<lslidar_msgs::srv::FrameRate::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::FrameRate::Response> res) {
                std::dynamic_pointer_cast<LslidarLsServices>(services_)->setFrameRate(req, res);
            });

        invalid_data_service = node_->create_service<lslidar_msgs::srv::InvalidData>("invalid_data",
            [this](std::shared_ptr<lslidar_msgs::srv::InvalidData::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::InvalidData::Response> res) {
                std::dynamic_pointer_cast<LslidarLsServices>(services_)->setInvalidData(req, res);
            });

        standby_mode_service = node_->create_service<lslidar_msgs::srv::StandbyMode>("standby_mode",
            [this](std::shared_ptr<lslidar_msgs::srv::StandbyMode::Request> req,
                   std::shared_ptr<lslidar_msgs::srv::StandbyMode::Response> res) {
                std::dynamic_pointer_cast<LslidarLsServices>(services_)->setStandbyMode(req, res);
            });
        
        if (!dump_file.empty()) {
            msop_input_.reset(new lslidar_driver::InputPCAP(node_, msop_udp_port, 1206, packet_rate, dump_file));
            difop_input_.reset(new lslidar_driver::InputPCAP(node_, difop_udp_port, 1206, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_driver::InputSocket(node_, msop_udp_port, 1206));
            difop_input_.reset(new lslidar_driver::InputSocket(node_, difop_udp_port, 1206));
        }

        thread_pool_->enqueue([this]() { difopPoll(); });
        
        return true;
    }

    void LslidarLsDriver::initTimeStamp() {
        for (unsigned char &i : this->packetTimeStamp) {
            i = 0;
        }
        this->pointcloudTimeStamp = 0;
    }
    
    bool LslidarLsDriver::initialize() {
        this->initTimeStamp();
        if (!loadParameters()) {
            LS_ERROR << "Cannot load all required ROS parameters..." << LS_END;
            return false;
        }

        this->outputParameters();

        if (!createRosIO()) {
            LS_ERROR << "Cannot create all ROS IO..." << LS_END;
            return false;
        }

        if (!getLidarInformation()) {
            LS_ERROR << "Cannot to obtain lidar configuration..." << LS_END;
            return false;
        }

        if (is_pretreatment) {
            pointcloud_transform_.setTransform(x_offset, y_offset, z_offset, roll, pitch, yaw);
        }

        // create the sin and cos table for different azimuth and vertical values
        for (int j = 0; j < 36000; ++j) {
            double angle = static_cast<double>(j) * 0.01 * 0.017453293;
            sin_table[j] = sin(angle);
            cos_table[j] = cos(angle);
        }

        double mirror_angle[4] = {1.5, -0.5, 0.5, -1.5};   //摆镜角度   //根据通道不同偏移角度不同
        for (int i = 0; i < 4; ++i) {
            cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle[i]));
            sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle[i]));
        }

        return true;
    }

    void LslidarLsDriver::difopPoll() {
        lslidar_msgs::msg::LslidarPacket::UniquePtr difop_packet(new lslidar_msgs::msg::LslidarPacket());

        while (rclcpp::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc == 0) {
                if (difop_packet->data[0] == 0x00 || difop_packet->data[0] == 0xa5) {
                    if (difop_packet->data[1] == 0xff && difop_packet->data[2] == 0x00 &&
                        difop_packet->data[3] == 0x5a) {

                        if (difop_packet->data[231] == 64 || difop_packet->data[231] == 65) {   // ls320 ls400
                            is_add_frame_ = true;
                        }

                        // 服务配置雷达传递设备包
                        services_->getDifopPacket(difop_packet);

                        m_horizontal_point = difop_packet->data[184] * 256 + difop_packet->data[185];

                        int majorVersion = difop_packet->data[1202];
                        int minorVersion1 = difop_packet->data[1203] / 16;
                        int minorVersion2 = difop_packet->data[1203] % 16;

                        //v1.1 :0.01   //v1.2以后  ： 0.0025
                        g_fAngleAcc_V = (1 > majorVersion || (1 == majorVersion && minorVersion1 > 1)) ? 0.0025 : 0.01;

                        std::ostringstream oss;
                        for (int i = 0; i < 4; ++i) {
                            std::bitset<8> bits(difop_packet->data[887 + i]);
                            oss << bits;
                            if (i < 3) oss << ' ';
                        }

                        auto fault_code_msg = std::make_shared<std_msgs::msg::String>();
                        fault_code_msg->data = oss.str();
                        fault_code_pub->publish(*fault_code_msg);

                        Information device = device_info_->getDeviceInfo(difop_packet);

                        lidar_info_data = std::make_shared<lslidar_msgs::msg::LslidarInformation>();
                        lidar_info_data->lidar_ip = device.lidarIp;
                        lidar_info_data->destination_ip = device.destinationIP;
                        lidar_info_data->lidar_mac_address = device.lidarMacAddress;
                        lidar_info_data->msop_port = device.msopPort;
                        lidar_info_data->difop_port = device.difopPort;
                        lidar_info_data->lidar_serial_number = device.lidarSerialNumber;
                        lidar_info_data->fpga_board_2_program = device.secondBoardProgram;
                        lidar_info_data->fpga_board_3_program = device.thirdBoardProgram;
                        lidar_info_pub->publish(*lidar_info_data);

                        is_get_difop_.store(true);
                    }
                }
            } else if (rc < 0) {
                return;
            }
        }
    }

    void LslidarLsDriver::publishPointCloudNew() {
        if (!is_get_difop_.load()) return;
        std::unique_lock<std::mutex> lock(pc_mutex_);
        point_cloud_xyzirt_pub_->header.frame_id = frame_id;
        point_cloud_xyzirt_pub_->height = 1;

        if (is_pretreatment) pointcloud_transform_.applyTransform(*point_cloud_xyzirt_pub_);

        auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();    
        pcl::toROSMsg(*point_cloud_xyzirt_pub_, *pc_msg);
        point_cloud_timestamp = (point_cloud_timestamp > 2147483647.0) ? (point_cloud_timestamp - 2208988800.0) : point_cloud_timestamp;
        pc_msg->header.stamp = rclcpp::Time(point_cloud_timestamp * 1000000000LL);
        pointcloud_pub->publish(*pc_msg);
        // LS_INFO << "pointcloud size: " << pc_msg->width << LS_END;

        auto time_msg = std::make_shared<std_msgs::msg::Float64>();
        time_msg->data = point_cloud_timestamp;
        time_pub->publish(*time_msg);
    }

    int LslidarLsDriver::convertCoordinate(const struct FiringLS &lidardata) {    
        double Angle_H = lidardata.azimuth;        
        double Angle_V = lidardata.vertical_angle; 

        if (Angle_H < 0.0) Angle_H += 360.0;
        if (Angle_V < 0.0) Angle_V += 360.0;

        int table_index_H = int(Angle_H * 100) % 36000;
        int table_index_V = int(Angle_V * 100) % 36000;

        const float x_coord = lidardata.distance * cos_table[table_index_V] * sin_table[table_index_H];
        const float y_coord = lidardata.distance * cos_table[table_index_V] * cos_table[table_index_H];
        const float z_coord = lidardata.distance * sin_table[table_index_V];

        VPoint point;
        point.x = x_coord;
        point.y = y_coord;
        point.z = z_coord;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.channel_number;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        point_cloud_xyzirt_bak_->points.push_back(point);
        ++point_cloud_xyzirt_bak_->width;

        return 0;
    }

    int LslidarLsDriver::convertCoordinateDistortion(const struct FiringLS &lidardata) {
        double fAngle_H = 0.0;     
        double fAngle_V = 0.0; 
        fAngle_H = lidardata.azimuth;
        fAngle_V = lidardata.vertical_angle;

        //加畸变
        double fSinV_angle = 0;
        double fCosV_angle = 0;

        //振镜偏移角度 = 实际垂直角度 / 2  - 偏移值
        double fGalvanometrtAngle = fAngle_V + m_offset;

        if (fGalvanometrtAngle < 0.0) fGalvanometrtAngle += 360.0;

        if (fAngle_H < 0.0) fAngle_H += 360.0;
        
        int table_index_V = int(fGalvanometrtAngle * 100) % 36000;
        int table_index_H = int(fAngle_H * 100) % 36000;

        double fAngle_R0 = cos30 * cos_mirror_angle[lidardata.channel_number % 4] * cos_table[table_index_V] -
                           sin_table[table_index_V] * sin_mirror_angle[lidardata.channel_number % 4];

        fSinV_angle = 2 * fAngle_R0 * sin_table[table_index_V] + sin_mirror_angle[lidardata.channel_number % 4];
        fCosV_angle = sqrt(1 - pow(fSinV_angle, 2));

        double fSinCite = (2 * fAngle_R0 * cos_table[table_index_V] * sin30 -
                           cos_mirror_angle[lidardata.channel_number % 4] * sin60) / fCosV_angle;
        double fCosCite = sqrt(1 - pow(fSinCite, 2));

        double fSinCite_H = sin_table[table_index_H] * fCosCite + cos_table[table_index_H] * fSinCite;
        double fCosCite_H = cos_table[table_index_H] * fCosCite - sin_table[table_index_H] * fSinCite;

        const float x_coord = lidardata.distance * fCosV_angle * fSinCite_H;
        const float y_coord = lidardata.distance * fCosV_angle * fCosCite_H;
        const float z_coord = lidardata.distance * fSinV_angle;

        VPoint point;
        point.x = x_coord;
        point.y = y_coord;
        point.z = z_coord;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.channel_number;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        point_cloud_xyzirt_bak_->points.push_back(point);
        ++point_cloud_xyzirt_bak_->width;

        return 0;
    }

    bool LslidarLsDriver::poll() {
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet(new lslidar_msgs::msg::LslidarPacket());

        while (true) {
            // keep reading until full packet received
            int rc = msop_input_->getPacket(packet);
            if (rc == 0) break;       // got a full packet?
            if (rc == 1) continue;    // No full packet, retry
            if (rc < 0) return false; // System-level error, terminate,end of file reached?
        }

        // publish message using time of last packet read
        if (use_time_service) {
            if (0xff == packet->data[1194]) {    //ptp授时
                uint64_t timestamp_s = (packet->data[1196] << 24) + (packet->data[1197] << 16) + 
                                       (packet->data[1198] << 8) + packet->data[1199];
                uint64_t timestamp_nsce = (packet->data[1200] << 24) + (packet->data[1201] << 16) +
                                          (packet->data[1202] << 8) + packet->data[1203];
                current_packet_time = rclcpp::Time(timestamp_s, timestamp_nsce).seconds();
            } else {          //gps授时
                this->packetTimeStamp[4] = packet->data[1199];
                this->packetTimeStamp[5] = packet->data[1198];
                this->packetTimeStamp[6] = packet->data[1197];
                this->packetTimeStamp[7] = packet->data[1196];
                this->packetTimeStamp[8] = packet->data[1195];
                this->packetTimeStamp[9] = packet->data[1194];
                struct tm cur_time{};
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_sec = this->packetTimeStamp[4];
                cur_time.tm_min = this->packetTimeStamp[5];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
                this->pointcloudTimeStamp = static_cast<uint64_t>(timegm(&cur_time)); //s
                uint64_t packet_timestamp;
                packet_timestamp = packet->data[1203] +
                                   (packet->data[1202] << 8) +
                                   (packet->data[1201] << 16) +
                                   (packet->data[1200] << 24); //ns
                current_packet_time = rclcpp::Time(this->pointcloudTimeStamp, packet_timestamp).seconds();
            }
        } else {
            current_packet_time = node_->get_clock()->now().seconds();
        }

        lslidarPacketProcess(packet);

        return true;
    }

    void LslidarLsDriver::packetProcessSingle(const lslidar_msgs::msg::LslidarPacket::UniquePtr &msg) {
        FiringLS lidardata;
        bool packetType = false;
        if (packet_loss) checkPacketLoss(msg, 1192, 2);
        double point_interval_time = (current_packet_time - last_packet_time) * SINGLE_ECHO;

        for (size_t point_idx = 0, point_num = 0; point_idx < POINTS_PER_PACKET_SINGLE_ECHO; point_idx += 8, ++point_num) {
            if ((msg->data[point_idx] == 0xff) && (msg->data[point_idx + 1] == 0xaa) && (msg->data[point_idx + 2] == 0xbb) && 
                (msg->data[point_idx + 3] == 0xcc) && (msg->data[point_idx + 4] == 0xdd)) {   // 帧头判断
                point_cloud_timestamp = last_packet_time + point_interval_time * point_num;
                packetType = true;
                frame_count++;
            } else {    //预计算点的距离
                double point_distance = ((msg->data[point_idx + 4] << 16) + (msg->data[point_idx + 5] << 8) +
                                            msg->data[point_idx + 6]) * g_fDistanceAcc;
                if (point_distance < min_range || point_distance > max_range) continue;
                memset(&lidardata, 0, sizeof(lidardata));
                //水平角度
                int fAngle_H = msg->data[point_idx + 1] + (msg->data[point_idx] << 8);
                if (fAngle_H > 32767) {
                    fAngle_H = (fAngle_H - 65536);
                }
                if ((fAngle_H < scan_start_angle) || (fAngle_H > scan_end_angle)) continue;
                lidardata.azimuth = fAngle_H * 0.01;
                //垂直角度+通道号
                int iTempAngle = msg->data[point_idx + 2];
                int iChannelNumber = iTempAngle >> 6; //左移六位 通道号
                int iSymmbol = (iTempAngle >> 5) & 0x01; //左移五位 符号位
                double fAngle_V = 0.0;
                if (1 == iSymmbol) // 符号位 0：正数 1：负数
                {
                    int iAngle_V = msg->data[point_idx + 3] + (msg->data[point_idx + 2] << 8);

                    fAngle_V = iAngle_V | 0xc000;
                    if (fAngle_V > 32767) {
                        fAngle_V = (fAngle_V - 65536);
                    }
                } else {
                    int iAngle_Hight = iTempAngle & 0x3f;
                    fAngle_V = msg->data[point_idx + 3] + (iAngle_Hight << 8);
                }

                lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
                lidardata.channel_number = iChannelNumber;
                lidardata.distance = point_distance;
                lidardata.intensity = msg->data[point_idx + 7];
                lidardata.time = last_packet_time + point_interval_time * (point_num + 1) - point_cloud_timestamp;
                lidarConvertCoordinate(lidardata);  // 计算坐标
            }

            if (packetType) {
                if (is_add_frame_) {
                    if (frame_count >= 2) {
                        {
                            std::unique_lock<std::mutex> lock(pc_mutex_);
                            point_cloud_xyzirt_pub_ = std::move(point_cloud_xyzirt_);
                        }
                        thread_pool_->enqueue([this]() { publishPointCloudNew(); });
                    }
                    packetType = false;
                    point_cloud_xyzirt_ = std::move(point_cloud_xyzirt_bak_);
                    point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>());
                } else {
                    {
                        std::unique_lock<std::mutex> lock(pc_mutex_);
                        point_cloud_xyzirt_pub_ = std::move(point_cloud_xyzirt_);
                    }
                    thread_pool_->enqueue([this]() { publishPointCloudNew(); });
                    packetType = false;
                    point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>());
                    point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>());
                }
            }
        }

        last_packet_time = current_packet_time;
    }

    //双回波数据包解析
    void LslidarLsDriver::packetProcessDouble(const lslidar_msgs::msg::LslidarPacket::UniquePtr &msg) {
        FiringLS lidardata;
        bool packetType = false;
        if (packet_loss) checkPacketLoss(msg, 1188, 6);
        double point_interval_time = (current_packet_time - last_packet_time) * DOUBLE_ECHO;

        for (size_t point_idx = 0, point_num = 0; point_idx < POINTS_PER_PACKET_DOUBLE_ECHO; point_idx += 12, ++point_num) {
            if ((msg->data[point_idx] == 0xff) && (msg->data[point_idx + 1] == 0xaa) && (msg->data[point_idx + 2] == 0xbb) && 
                (msg->data[point_idx + 3] == 0xcc) && (msg->data[point_idx + 4] == 0xdd)) {
                point_cloud_timestamp = last_packet_time + point_interval_time * point_num;
                packetType = true;
                frame_count++;
            } else {
                double point_distance1 = ((msg->data[point_idx + 4] << 16) + (msg->data[point_idx + 5] << 8) +
                                           msg->data[point_idx + 6]) * g_fDistanceAcc;
                if (point_distance1 < min_range || point_distance1 > max_range) continue;
                double point_distance2 = ((msg->data[point_idx + 8] << 16) + (msg->data[point_idx + 9] << 8) +
                                           msg->data[point_idx + 10]) * g_fDistanceAcc;
                memset(&lidardata, 0, sizeof(lidardata));
                //水平角度
                int fAngle_H = msg->data[point_idx + 1] + (msg->data[point_idx] << 8);
                if (fAngle_H > 32767) {
                    fAngle_H = (fAngle_H - 65536);
                }
                if ((fAngle_H < scan_start_angle) || (fAngle_H > scan_end_angle)) continue;
                //垂直角度+通道号
                int iTempAngle = msg->data[point_idx + 2];
                int iChannelNumber = iTempAngle >> 6; //左移六位 通道号
                int iSymmbol = (iTempAngle >> 5) & 0x01; //左移五位 符号位
                double fAngle_V = 0.0;
                if (1 == iSymmbol) // 符号位 0：正数 1：负数
                {
                    int iAngle_V = msg->data[point_idx + 3] + (msg->data[point_idx + 2] << 8);

                    fAngle_V = iAngle_V | 0xc000;
                    if (fAngle_V > 32767) {
                        fAngle_V = (fAngle_V - 65536);
                    }
                } else {
                    int iAngle_Hight = iTempAngle & 0x3f;
                    fAngle_V = msg->data[point_idx + 3] + (iAngle_Hight << 8);
                }

                lidardata.azimuth = fAngle_H * 0.01;
                lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
                lidardata.channel_number = iChannelNumber;
                lidardata.distance = point_distance1;
                lidardata.intensity = msg->data[point_idx + 7];
                lidardata.time = last_packet_time + point_interval_time * (point_num + 1) - point_cloud_timestamp;
                lidarConvertCoordinate(lidardata);  // 第一个点

                lidardata.distance = point_distance2;
                if (lidardata.distance < min_range || lidardata.distance > max_range) continue;
                lidardata.intensity = msg->data[point_idx + 11];
                lidarConvertCoordinate(lidardata);  // 第二个点
            }

            if (packetType) {
                if (is_add_frame_) {
                    if (frame_count >= 2) {
                        {
                            std::unique_lock<std::mutex> lock(pc_mutex_);
                            point_cloud_xyzirt_pub_ = std::move(point_cloud_xyzirt_);
                        }
                        thread_pool_->enqueue([this]() { publishPointCloudNew(); });
                    }
                    packetType = false;
                    point_cloud_xyzirt_ = std::move(point_cloud_xyzirt_bak_);
                    point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>());
                } else {
                    {
                        std::unique_lock<std::mutex> lock(pc_mutex_);
                        point_cloud_xyzirt_pub_ = std::move(point_cloud_xyzirt_);
                    }
                    thread_pool_->enqueue([this]() { publishPointCloudNew(); });
                    packetType = false;
                    point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>());
                    point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>());
                }
            }
        }

        last_packet_time = current_packet_time;
    }

    void LslidarLsDriver::checkPacketLoss(const lslidar_msgs::msg::LslidarPacket::UniquePtr &msg, int data_offset, int byte_count) {
        int64_t current_packet_number_ = 0;

        for (int i = 0; i < byte_count; ++i) {
            current_packet_number_ = (current_packet_number_ << 8) + msg->data[data_offset + i];
        }

        tmp_packet_number_ = current_packet_number_;

        if(current_packet_number_ - last_packet_number_ < 0){current_packet_number_ += 65536;}

        if (current_packet_number_ - last_packet_number_ > 1  && last_packet_number_ != -1) {
            // if(current_packet_number_ - last_packet_number_ > 20) LS_INFO << "num: " << current_packet_number_ - last_packet_number_ << "  98-03: " << msg->data[1198] << msg->data[1199] << msg->data[1200] << msg->data[1201] << msg->data[1202] << msg->data[1203] << LS_END;
            total_packet_loss_ += current_packet_number_ - last_packet_number_ - 1;

            auto loss_data = std::make_shared<std_msgs::msg::Int64>();
            loss_data->data = total_packet_loss_;
            packet_loss_pub->publish(*loss_data);
        }
        last_packet_number_ = tmp_packet_number_;
    }

    bool LslidarLsDriver::getLidarInformation(){
        lslidar_msgs::msg::LslidarPacket::UniquePtr msg(new lslidar_msgs::msg::LslidarPacket());
    
        while (true) {
            // keep reading until full packet received
            int rc_ = msop_input_->getPacket(msg);

            if (rc_ == 0) break;       // got a full packet?
            if (rc_ < 0) return false; // end of file reached?
        }
        
        if(get_ms06_param && m_horizontal_point != 0 && msg->data[1204] == 192){
            //ms06  param
            double mirror_angle[4] = {1.5, 0.5, -0.5, -1.5};
            for (int i = 0; i < 4; ++i) {
                cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle[i]));
                sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle[i]));
            }
            m_offset = 10.82;
            g_fAngleAcc_V = 0.01;
            g_fDistanceAcc = 0.004;
            get_ms06_param = false;
            LS_INFO << "Lidar type MS06" << LS_END;
        }

        if(msg->data[1205] == 0x01 || msg->data[1205] == 0x11) {
            lslidarPacketProcess = std::bind(&LslidarLsDriver::packetProcessSingle, this, std::placeholders::_1);
            if (msg->data[1205] == 0x01) {
                lidarConvertCoordinate = std::bind(&LslidarLsDriver::convertCoordinateDistortion, this, std::placeholders::_1);
                LS_INFO << "Lidar distortion correction off" << LS_END;
            } else if (msg->data[1205] == 0x11) {
                lidarConvertCoordinate = std::bind(&LslidarLsDriver::convertCoordinate, this, std::placeholders::_1);
                LS_INFO << "Lidar distortion correction on" << LS_END;
            }
            LS_INFO << "Lidar echo mode: single echo" << LS_END;
        } else if(msg->data[1205] == 0x02 || msg->data[1205] == 0x12){
            lslidarPacketProcess = std::bind(&LslidarLsDriver::packetProcessDouble, this, std::placeholders::_1);
            if (msg->data[1205] == 0x02) {
                lidarConvertCoordinate = std::bind(&LslidarLsDriver::convertCoordinateDistortion, this, std::placeholders::_1);
                LS_INFO << "Lidar distortion correction off" << LS_END;
            } else if (msg->data[1205] == 0x12) {
                lidarConvertCoordinate = std::bind(&LslidarLsDriver::convertCoordinate, this, std::placeholders::_1);
                LS_INFO << "Lidar distortion correction on" << LS_END;
            }
            LS_INFO << "Lidar echo mode: double echo" << LS_END;
        } else {
            return false;
        }

        return true;
    };

} // namespace lslidar_driver
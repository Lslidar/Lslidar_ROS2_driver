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

#include "lslidar_ls_driver/lslidar_driver.h"


namespace lslidar_driver {
    using namespace std::chrono;

    lslidarDriver::lslidarDriver() : lslidarDriver(rclcpp::NodeOptions()) {
    }

    lslidarDriver::lslidarDriver(const rclcpp::NodeOptions &options) : Node("lslidar_node", options),
                                                                       socket_id(-1),
                                                                       time_synchronization(false),
                                                                       min_range(0.3),
                                                                       max_range(500),
                                                                       packet_rate(11111.0),
                                                                       packet_end_time(0.0),
                                                                       current_packet_time(0.0),
                                                                       last_packet_time(0.0),
                                                                       return_mode(1),
                                                                       point_time(0.0),
                                                                       g_fAngleAcc_V(0.01),
                                                                       is_add_frame_(false),
                                                                       is_get_difop_(false),
                                                                       packet_loss(false),
                                                                       get_ms06_param(true),
                                                                       last_packet_number_(-1),
                                                                       total_packet_loss_(0),
                                                                       frame_count(0),
                                                                       threadPool_(std::make_unique<ThreadPool>(2)),
                                                                       point_cloud_xyzirt_(new pcl::PointCloud<VPoint>),
                                                                       point_cloud_xyzirt_bak_(new pcl::PointCloud<VPoint>),
                                                                       point_cloud_xyzirt_pub_(new pcl::PointCloud<VPoint>) {
        // create the sin and cos table for different azimuth and vertical values
        for (int j = 0; j < 36000; ++j) {
            double angle = static_cast<double>(j) / 100.0 * M_PI / 180.0;
            sin_table[j] = sin(angle);
            cos_table[j] = cos(angle);
        }

        double mirror_angle[4] = {1.5, -0.5, 0.5, -1.5};   //摆镜角度   //根据通道不同偏移角度不同
        for (int i = 0; i < 4; ++i) {
            cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle[i]));
            sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle[i]));
        }
    }

    lslidarDriver::~lslidarDriver() {
        if (nullptr == difop_thread_) {
            difop_thread_->join();
        }

        (void) close(socket_id);
    }

    bool lslidarDriver::loadParameters() {
        this->declare_parameter<std::string>("pcap", "");
        this->declare_parameter<double>("packet_rate", 11111.0);
        this->declare_parameter<std::string>("frame_id", "laser_link");
        this->declare_parameter<std::string>("device_ip", "192.168.1.200");
        this->declare_parameter<bool>("add_multicast", false);
        this->declare_parameter<std::string>("group_ip", "234.2.3.2");
        this->declare_parameter<int>("msop_port", 2368);
        this->declare_parameter<int>("difop_port", 2369);
        this->declare_parameter<bool>("time_synchronization", false);
        this->declare_parameter<double>("min_range", 0.3);
        this->declare_parameter<double>("max_range", 150.0);
        this->declare_parameter<int>("scan_start_angle", -60);
        this->declare_parameter<int>("scan_end_angle", 60);
        this->declare_parameter<std::string>("topic_name", "lslidar_point_cloud");
        this->declare_parameter<bool>("packet_loss", false);
        this->declare_parameter<bool>("output_time_source", false);

        this->get_parameter("pcap", dump_file);
        this->get_parameter("packet_rate", packet_rate);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("add_multicast", add_multicast);
        this->get_parameter("group_ip", group_ip_string);
        this->get_parameter("device_ip", lidar_ip_string);
        this->get_parameter("msop_port", msop_udp_port);
        this->get_parameter("difop_port", difop_udp_port);
        this->get_parameter("min_range", min_range);
        this->get_parameter("max_range", max_range);
        this->get_parameter("scan_start_angle", scan_start_angle);
        this->get_parameter("scan_end_angle", scan_end_angle);
        this->get_parameter("time_synchronization", time_synchronization);
        this->get_parameter("topic_name", pointcloud_topic);
        this->get_parameter("packet_loss", packet_loss);
        this->get_parameter("output_time_source", output_time_source);

        return true;
    }

    void lslidarDriver::outputParameters() {
        LS_PARAM << "******** LS1550 ROS2 driver version: 1.0.5 ********" << LS_END;
        LS_PARAM << "dump file: " << dump_file << LS_END;
        LS_PARAM << "packet rate: " << packet_rate << LS_END;
        LS_PARAM << "add multicast: " << std::boolalpha << add_multicast << LS_END;
        LS_PARAM << "use time service: " << std::boolalpha << time_synchronization << LS_END;
        LS_PARAM << "packet loss check: " << std::boolalpha << packet_loss << LS_END;
        LS_PARAM << "min range: " << min_range << LS_END;
        LS_PARAM << "max range: "<< max_range << LS_END;
        LS_PARAM << "scan start angle: " << scan_start_angle << LS_END;
        LS_PARAM << "scan end angle: " << scan_end_angle << LS_END;
        LS_PARAM << "frame id: " << frame_id << LS_END;
        LS_PARAM << "pointcloud topic: " << pointcloud_topic << LS_END;

        LS_SOCKET << "Accepting packets from IP address: " << lidar_ip_string.c_str() << LS_END;
        if (add_multicast) LS_SOCKET << "Opening UDP socket: group address: " << group_ip_string.c_str() << LS_END;
        LS_SOCKET << "Opening UDP socket msop port: " << msop_udp_port << LS_END;
        LS_SOCKET << "Opening UDP socket difop port: " << difop_udp_port << LS_END;
    }

    bool lslidarDriver::createRosIO() {
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
        fault_code_pub = this->create_publisher<std_msgs::msg::String>("lslidar_fault_code", 1);
        if (packet_loss) packet_loss_pub_ = this->create_publisher<std_msgs::msg::Int64>("packet_loss", 1000);
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        angle_distortion_correction_service_ = this->create_service<lslidar_ls_driver::srv::AngleDistortionCorrection>("set_angle_distortion_correction",
                                                                                std::bind(
                                                                                        &lslidarDriver::setAngleDistortionCorrection,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2),
                                                                                qos_profile);
        data_ip_service_ = this->create_service<lslidar_ls_driver::srv::DataIp>("set_data_ip",
                                                                                std::bind(
                                                                                        &lslidarDriver::setDataIp,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2),
                                                                                qos_profile);
        data_port_service_ = this->create_service<lslidar_ls_driver::srv::DataPort>("set_data_port",
                                                                                std::bind(
                                                                                        &lslidarDriver::setDataPort,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2),
                                                                                qos_profile);                                                                        
        destination_ip_service_ = this->create_service<lslidar_ls_driver::srv::DestinationIp>("set_destination_ip",
                                                                                std::bind(
                                                                                        &lslidarDriver::setDestinationIp,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2),
                                                                                qos_profile);
        dev_port_service_ = this->create_service<lslidar_ls_driver::srv::DevPort>("set_dev_port",
                                                                                std::bind(
                                                                                        &lslidarDriver::setDevPort,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2),
                                                                                qos_profile);
        frame_rate_service_ = this->create_service<lslidar_ls_driver::srv::FrameRate>("set_frame_rate",
                                                                                std::bind(
                                                                                        &lslidarDriver::setFrameRate,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2),
                                                                                qos_profile);
        invalid_data_service_ = this->create_service<lslidar_ls_driver::srv::InvalidData>("set_invalid_data",
                                                                                std::bind(
                                                                                        &lslidarDriver::setInvalidData,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2),
                                                                                qos_profile);
        standby_mode_service_ = this->create_service<lslidar_ls_driver::srv::StandbyMode>("set_standby_mode",
                                                                                std::bind(
                                                                                        &lslidarDriver::setStandbyMode,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2),
                                                                                qos_profile);                                                                   
        time_service_ = this->create_service<lslidar_ls_driver::srv::TimeService>("set_time_service",
                                                                                std::bind(
                                                                                        &lslidarDriver::setTimeService,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2),
                                                                                qos_profile);

        if (!dump_file.empty()) {
            msop_input_.reset(new lslidar_driver::InputPCAP(this, msop_udp_port, packet_rate, dump_file));
            difop_input_.reset(new lslidar_driver::InputPCAP(this, difop_udp_port, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_driver::InputSocket(this, msop_udp_port));
            difop_input_.reset(new lslidar_driver::InputSocket(this, difop_udp_port));
        }

        difop_thread_ = std::make_shared<std::thread>([this]() { difopPoll(); });

        return true;
    }

    void lslidarDriver::initTimeStamp() {
        for (unsigned char &i : this->packetTimeStamp) {
            i = 0;
        }
        this->pointcloudTimeStamp = 0;
        this->timeStamp = rclcpp::Time(0.0);
    }

    bool lslidarDriver::initialize() {
        this->initTimeStamp();
        if (!loadParameters()) {
            LS_ERROR << "Cannot load all required ROS parameters..." << LS_END;
            return false;
        }

        if (!createRosIO()) {
            LS_ERROR << "Cannot create all ROS IO..." << LS_END;
            return false;
        }

        outputParameters();

        if (!getLidarInformation()) {
            LS_ERROR << "Cannot to obtain lidar configuration..." << LS_END;
            return false;
        }

        poll_thread_ = std::make_shared<std::thread>(&lslidarDriver::pollThread, this);

        return true;
    }

    bool lslidarDriver::polling() {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_ls_driver::msg::LslidarLsPacket::UniquePtr packet(new lslidar_ls_driver::msg::LslidarLsPacket());
        // Since the lslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
            while (true) {
                // keep reading until full packet received
                int rc = msop_input_->getPacket(packet);
                if (rc == 0) break;       // got a full packet?
                if (rc < 0) return false; // end of file reached?
            }

            if (time_synchronization) {
                // it is already the msop msg
                // use the first packets
                lslidar_ls_driver::msg::LslidarLsPacket pkt = *packet;
                if (0xff == pkt.data[1194]) {    //ptp授时
                    int64_t timestamp_s = ((static_cast<int64_t>(pkt.data[1195]) << 32) + (static_cast<int64_t>(pkt.data[1196]) << 24) +
                                           (static_cast<int64_t>(pkt.data[1197]) << 16) + (static_cast<int64_t>(pkt.data[1198]) << 8) +
                                            static_cast<int64_t>(pkt.data[1199]));

                    int64_t timestamp_nsce = ((static_cast<int64_t>(pkt.data[1200]) << 24) + (static_cast<int64_t>(pkt.data[1201]) << 16) +
                                              (static_cast<int64_t>(pkt.data[1202]) << 8) + static_cast<int64_t>(pkt.data[1203]));

                    int64_t total_nanoseconds = timestamp_s * 1000000000LL + timestamp_nsce;

                    timeStamp = rclcpp::Time(total_nanoseconds);
                    packet->stamp = timeStamp;
                    current_packet_time = timeStamp.seconds();
                } else {          //gps授时
                    this->packetTimeStamp[4] = pkt.data[1199];
                    this->packetTimeStamp[5] = pkt.data[1198];
                    this->packetTimeStamp[6] = pkt.data[1197];
                    this->packetTimeStamp[7] = pkt.data[1196];
                    this->packetTimeStamp[8] = pkt.data[1195];
                    this->packetTimeStamp[9] = pkt.data[1194];
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
                    packet_timestamp = pkt.data[1203] + (pkt.data[1202] << 8) + (pkt.data[1201] << 16) + (pkt.data[1200] << 24); //ns

                    timeStamp = rclcpp::Time(pointcloudTimeStamp, packet_timestamp);  // s,ns
                    packet->stamp = timeStamp;
                    current_packet_time = timeStamp.seconds();
                }
            } else {
                packet->stamp = get_clock()->now();
                current_packet_time = rclcpp::Time(packet->stamp).seconds();
            }

            lslidarPacketProcess(packet);

            return true;
    }

    void lslidarDriver::difopPoll() {
        lslidar_ls_driver::msg::LslidarLsPacket::UniquePtr difop_packet(new lslidar_ls_driver::msg::LslidarLsPacket());
        // reading and publishing scans as fast as possible.
        while (rclcpp::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc == 0) {
                if (difop_packet->data[0] == 0x00 || difop_packet->data[0] == 0xa5) {
                    if (difop_packet->data[1] == 0xff && difop_packet->data[2] == 0x00 &&
                        difop_packet->data[3] == 0x5a) {
                        
                        if (output_time_source) {
                            if (time_synchronization) {
                                if (difop_packet->data[44] == 0) LS_INFO << "Lidar usage GPS time." << LS_END;
                                if (difop_packet->data[44] == 1) LS_INFO << "Lidar usage PTP(L2) time." << LS_END;
                                if (difop_packet->data[44] == 2) LS_INFO << "Lidar usage NTP time." << LS_END;
                                if (difop_packet->data[44] == 3) LS_INFO << "Lidar usage PTP(UDPV4) time." << LS_END;
                            } else {
                                LS_INFO << "Lidar usage system time." << LS_END;
                            }
                            output_time_source = false;
                        }
                        
                        if (difop_packet->data[231] == 64 || difop_packet->data[231] == 65)  is_add_frame_ = true;  // ls320 ls400

                        for (int i = 0; i < 1206; i++) {
                            difop_data[i] = difop_packet->data[i];
                        }

                        m_horizontal_point = difop_packet->data[184] * 256 + difop_packet->data[185];
                        int majorVersion = difop_packet->data[1202];
                        int minorVersion1 = difop_packet->data[1203] / 16;
                        // int minorVersion2 = difop_packet->data[1203] % 16;

                        //v1.1 :0.01   //v1.2以后  ： 0.0025
                        g_fAngleAcc_V = (1 > majorVersion || (1 == majorVersion && minorVersion1 > 1)) ? 0.0025 : 0.01;

                        auto process_angle = [&difop_packet, this](int index) {
                            float fInitAngle_V = difop_packet->data[index] * 256 + difop_packet->data[index + 1];
                            if (fInitAngle_V > 32767) {
                                fInitAngle_V -= 65536;
                            }
                            return fInitAngle_V * g_fAngleAcc_V;
                        };

                        this->prism_angle[0] = process_angle(188);
                        this->prism_angle[1] = process_angle(190);
                        this->prism_angle[2] = process_angle(192);
                        this->prism_angle[3] = process_angle(194);

                        std::ostringstream oss;
    
                        for (int i = 0; i < 4; ++i) {
                            std::bitset<8> bits(difop_packet->data[887 + i]);
                            
                            oss << bits;
                            
                            if (i < 3) oss << ' ';
                        }

                        std_msgs::msg::String fault_code_msg;
                        fault_code_msg.data = oss.str();
                        fault_code_pub->publish(fault_code_msg);

                        is_get_difop_ = true;
                    }
                }
            } else if (rc < 0) {
                return;
            }
        }
    }

    void lslidarDriver::publishPointCloud() {
        if (!is_get_difop_) return;
        std::unique_lock<std::mutex> lock(pc_lock);
        point_cloud_xyzirt_pub_->header.frame_id = frame_id;
        point_cloud_xyzirt_pub_->height = 1;
        sensor_msgs::msg::PointCloud2 pc_msg;
        pcl::toROSMsg(*point_cloud_xyzirt_pub_, pc_msg);
        point_cloud_timestamp = (point_cloud_timestamp > 2147483647.0) ? (point_cloud_timestamp - 2208988800.0) : point_cloud_timestamp;
        // RCLCPP_INFO(this->get_logger(), "point_cloud_timestamp: %.9f", point_cloud_timestamp);
        pc_msg.header.stamp = rclcpp::Time(point_cloud_timestamp * 1000000000LL);
        pointcloud_pub_->publish(pc_msg);
        // RCLCPP_INFO(this->get_logger(), "pointcloud size: %u", pc_msg.width);
    }

    int lslidarDriver::convertCoordinate(const struct Firing &lidardata) {    
        double Angle_H = lidardata.azimuth;        
        double Angle_V = lidardata.vertical_angle; 

        if (Angle_H < 0.0) Angle_H += 360.0;
        if (Angle_V < 0.0) Angle_V += 360.0;

        int table_index_H = int(Angle_H * 100) % 36000;
        int table_index_V = int(Angle_V * 100) % 36000;

        const float x_coord = lidardata.distance * cos_table[table_index_V] * sin_table[table_index_H];
        const float y_coord = lidardata.distance * cos_table[table_index_V] * cos_table[table_index_H];
        const float z_coord = lidardata.distance * sin_table[table_index_V];

        point_cloud_xyzirt_->points.emplace_back(x_coord, y_coord, z_coord, lidardata.intensity, lidardata.channel_number, lidardata.time);
        ++point_cloud_xyzirt_->width;

        point_cloud_xyzirt_bak_->points.emplace_back(x_coord, y_coord, z_coord, lidardata.intensity, lidardata.channel_number, lidardata.time);
        ++point_cloud_xyzirt_bak_->width;

        return 0;
    }

    int lslidarDriver::convertCoordinateDistortion(const struct Firing &lidardata) {
        double fAngle_H = 0.0;         //水平角度
        double fAngle_V = 0.0;        // 垂直角度
        fAngle_H = lidardata.azimuth;
        fAngle_V = lidardata.vertical_angle;

        //加畸变
        double fSinV_angle = 0;
        double fCosV_angle = 0;

        //振镜偏移角度 = 实际垂直角度 / 2  - 偏移值
        double fGalvanometrtAngle = fAngle_V + m_offset;

        while (fGalvanometrtAngle < 0.0) {
            fGalvanometrtAngle += 360.0;
        }

        while (fAngle_H < 0.0) {
            fAngle_H += 360.0;
        }

        int table_index_V = int(fGalvanometrtAngle * 100) % 36000;
        int table_index_H = int(fAngle_H * 100) % 36000;

        double fAngle_R0 = cos30 * cos_mirror_angle[lidardata.channel_number % 4] * cos_table[table_index_V] -
                           sin_table[table_index_V] * sin_mirror_angle[lidardata.channel_number % 4];   //mode 4 确保不会超出数组范围

        fSinV_angle = 2 * fAngle_R0 * sin_table[table_index_V] + sin_mirror_angle[lidardata.channel_number % 4];
        fCosV_angle = sqrt(1 - pow(fSinV_angle, 2));

        double fSinCite = (2 * fAngle_R0 * cos_table[table_index_V] * sin30 -
                           cos_mirror_angle[lidardata.channel_number % 4] * sin60) / fCosV_angle;
        double fCosCite = sqrt(1 - pow(fSinCite, 2));

        double fSinCite_H = sin_table[table_index_H] * fCosCite + cos_table[table_index_H] * fSinCite;
        double fCosCite_H = cos_table[table_index_H] * fCosCite - sin_table[table_index_H] * fSinCite;

        double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;
        x_coord = lidardata.distance * fCosV_angle * fSinCite_H;
        y_coord = lidardata.distance * fCosV_angle * fCosCite_H;
        z_coord = lidardata.distance * fSinV_angle;

        point_cloud_xyzirt_->points.emplace_back(x_coord, y_coord, z_coord, lidardata.intensity, lidardata.channel_number, lidardata.time);
        ++point_cloud_xyzirt_->width;

        point_cloud_xyzirt_bak_->points.emplace_back(x_coord, y_coord, z_coord, lidardata.intensity, lidardata.channel_number, lidardata.time);
        ++point_cloud_xyzirt_bak_->width;

        return 0;
    }

    void lslidarDriver::packetProcessSingle(const lslidar_ls_driver::msg::LslidarLsPacket::UniquePtr &msg) {
        Firing lidardata;
        bool packetType = false;
        if (packet_loss) checkPacketLoss(msg, 1192, 2);
        double point_interval_time = (current_packet_time - last_packet_time) * SINGLE_ECHO;

        for (size_t point_idx = 0, point_num = 0; point_idx < POINTS_PER_PACKET_SINGLE_ECHO; point_idx += 8, ++point_num) {
            if ((msg->data[point_idx] == 0xff) && (msg->data[point_idx + 1] == 0xaa) &&
                (msg->data[point_idx + 2] == 0xbb) && (msg->data[point_idx + 3] == 0xcc)) {
                point_cloud_timestamp = last_packet_time + point_interval_time * point_num;
                point_cloud_timestamp = point_cloud_timestamp > 0 ? point_cloud_timestamp : 0;
                packetType = true;
                frame_count++;
            } else {    
                double point_distance = ((msg->data[point_idx + 4] << 16) + (msg->data[point_idx + 5] << 8) +
                                            msg->data[point_idx + 6]) * g_fDistanceAcc;
                if (point_distance < min_range || point_distance > max_range) continue;
                memset(&lidardata, 0, sizeof(lidardata));

                //水平角度
                double fAngle_H = msg->data[point_idx + 1] + (msg->data[point_idx] << 8);
                if (fAngle_H > 32767) {
                    fAngle_H = (fAngle_H - 65536);
                }
                lidardata.azimuth = fAngle_H * 0.01;
                if ((lidardata.azimuth < scan_start_angle) || (lidardata.azimuth > scan_end_angle)) continue;
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
                            std::unique_lock<std::mutex> lock(pc_lock);
                            point_cloud_xyzirt_pub_ = std::move(point_cloud_xyzirt_);
                        }
                        threadPool_->enqueue([&]() { publishPointCloud(); });
                    }
                    packetType = false;
                    point_cloud_xyzirt_ = std::move(point_cloud_xyzirt_bak_);
                    point_cloud_xyzirt_bak_ = pcl::make_shared<pcl::PointCloud<VPoint>>();
                } else {
                    {
                        std::unique_lock<std::mutex> lock(pc_lock);
                        point_cloud_xyzirt_pub_ = std::move(point_cloud_xyzirt_);
                    }
                    threadPool_->enqueue([&]() { publishPointCloud(); });
                    packetType = false;
                    point_cloud_xyzirt_ = pcl::make_shared<pcl::PointCloud<VPoint>>();
                    point_cloud_xyzirt_bak_ = pcl::make_shared<pcl::PointCloud<VPoint>>();
                }
            }
        }

        last_packet_time = current_packet_time;
    }

    void lslidarDriver::packetProcessDouble(const lslidar_ls_driver::msg::LslidarLsPacket::UniquePtr &msg) {
        struct Firing lidardata{};
        bool packetType = false;
        if (packet_loss) checkPacketLoss(msg, 1188, 6);
        point_interval_time = packet_interval_time * DOUBLE_ECHO;

        for (size_t point_idx = 0, point_num = 0; point_idx < POINTS_PER_PACKET_DOUBLE_ECHO; point_idx += 12, ++point_num) {
            if ((msg->data[point_idx] == 0xff) && (msg->data[point_idx + 1] == 0xaa) &&
                (msg->data[point_idx + 2] == 0xbb) && (msg->data[point_idx + 3] == 0xcc)) {
                point_cloud_timestamp = last_packet_time + point_interval_time * point_num;//precomputed_point_times[point_num];
                point_cloud_timestamp = point_cloud_timestamp > 0 ? point_cloud_timestamp : 0;
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
                double fAngle_H = msg->data[point_idx + 1] + (msg->data[point_idx] << 8);
                if (fAngle_H > 32767) {
                    fAngle_H = (fAngle_H - 65536);
                }
                
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
                if ((lidardata.azimuth < scan_start_angle) || (lidardata.azimuth > scan_end_angle)) continue;
                lidardata.channel_number = iChannelNumber;
                lidardata.distance = point_distance1;
                lidardata.intensity = msg->data[point_idx + 7];
                lidardata.time = last_packet_time + point_interval_time * (point_num + 1) - point_cloud_timestamp;//precomputed_point_times[point_num + 1] - point_cloud_timestamp;
                lidarConvertCoordinate(lidardata);

                lidardata.distance = point_distance2;
                if (lidardata.distance < min_range || lidardata.distance > max_range) continue;
                lidardata.intensity = msg->data[point_idx + 11];
                lidarConvertCoordinate(lidardata);
            }

            if (packetType) {
                if (is_add_frame_) {
                    if (frame_count >= 2) {
                        {
                            std::unique_lock<std::mutex> lock(pc_lock);
                            point_cloud_xyzirt_pub_ = std::move(point_cloud_xyzirt_);
                        }
                        threadPool_->enqueue([&]() { publishPointCloud(); });
                    }
                    packetType = false;
                    point_cloud_xyzirt_ = std::move(point_cloud_xyzirt_bak_);
                    point_cloud_xyzirt_bak_ = pcl::make_shared<pcl::PointCloud<VPoint>>();
                } else {
                    {
                        std::unique_lock<std::mutex> lock(pc_lock);
                        point_cloud_xyzirt_pub_ = std::move(point_cloud_xyzirt_);
                    }
                    threadPool_->enqueue([&]() { publishPointCloud(); });
                    packetType = false;
                    point_cloud_xyzirt_ = pcl::make_shared<pcl::PointCloud<VPoint>>();
                    point_cloud_xyzirt_bak_ = pcl::make_shared<pcl::PointCloud<VPoint>>();
                }
            }
        }
    }

    void lslidarDriver::checkPacketLoss(const lslidar_ls_driver::msg::LslidarLsPacket::UniquePtr &msg, int data_offset, int byte_count) {
        int64_t current_packet_number_ = 0;

        for (int i = 0; i < byte_count; ++i) {
            current_packet_number_ = (current_packet_number_ << 8) + msg->data[data_offset + i];
        }

        tmp_packet_number_ = current_packet_number_;

        if(current_packet_number_ - last_packet_number_ < 0){current_packet_number_ += 65536;}

        if (current_packet_number_ - last_packet_number_ > 1  && last_packet_number_ != -1) {
            RCLCPP_WARN(this->get_logger(),"error!: 1188-1193: %02x %02x %02x %02x %02x %02x", msg->data[1188], msg->data[1189], msg->data[1190], msg->data[1191], msg->data[1192], msg->data[1193]);
            total_packet_loss_ += current_packet_number_ - last_packet_number_ - 1;
            std_msgs::msg::Int64 loss_data;
            loss_data.data = total_packet_loss_;
            packet_loss_pub_->publish(loss_data);
        }
        last_packet_number_ = tmp_packet_number_;
    }

    void lslidarDriver::pollThread(void) {
        while (rclcpp::ok()) {
            polling();
        }
    }

    bool lslidarDriver::getLidarInformation(){
        lslidar_ls_driver::msg::LslidarLsPacket::UniquePtr msg(new lslidar_ls_driver::msg::LslidarLsPacket());

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
        }

        if(msg->data[1205] == 0x01 || msg->data[1205] == 0x11) {
            lslidarPacketProcess = std::bind(&lslidarDriver::packetProcessSingle, this, std::placeholders::_1);
            if (msg->data[1205] == 0x01) {
                lidarConvertCoordinate = std::bind(&lslidarDriver::convertCoordinateDistortion, this, std::placeholders::_1);
                LS_INFO << "Lidar angle distortion correction off" << LS_END;
            } else if (msg->data[1205] == 0x11) {
                lidarConvertCoordinate = std::bind(&lslidarDriver::convertCoordinate, this, std::placeholders::_1);
                LS_INFO << "Lidar angle distortion correction on" << LS_END ;
            }
            LS_INFO << "Lidar echo mode: single echo" << LS_END;
        } else if(msg->data[1205] == 0x02 || msg->data[1205] == 0x12){
            lslidarPacketProcess = std::bind(&lslidarDriver::packetProcessDouble, this, std::placeholders::_1);
            if (msg->data[1205] == 0x02) {
                lidarConvertCoordinate = std::bind(&lslidarDriver::convertCoordinateDistortion, this, std::placeholders::_1);
                LS_INFO << "Lidar angle distortion correction off" << LS_END;
            } else if (msg->data[1205] == 0x12) {
                lidarConvertCoordinate = std::bind(&lslidarDriver::convertCoordinate, this, std::placeholders::_1);
                LS_INFO << "Lidar angle distortion correction on" << LS_END;
            }
            LS_INFO << "Lidar echo mode: double echo" << LS_END;
        } else {
            return false;
        }

        std::cout << std::endl;

        return true;
    }

    void lslidarDriver::setPacketHeader(unsigned char *config_data) {
        config_data[0] = 0xAA;
        config_data[1] = 0x00;
        config_data[2] = 0xFF;
        config_data[3] = 0x11;
        config_data[4] = 0x22;
        config_data[5] = 0x22;
        config_data[6] = 0xAA;
        config_data[7] = 0xAA;
    }

    bool lslidarDriver::sendPacketTolidar(unsigned char *config_data) const {
        int socketid;
        sockaddr_in addrSrv{};
        socketid = socket(2, 2, 0);
        addrSrv.sin_addr.s_addr = inet_addr(lidar_ip_string.c_str());
        addrSrv.sin_family = AF_INET;
        addrSrv.sin_port = htons(2368);
        sendto(socketid, (const char *) config_data, 1206, 0, (struct sockaddr *) &addrSrv, sizeof(addrSrv));
        close(socketid);

        return true;
    }

    bool lslidarDriver::setAngleDistortionCorrection(std::shared_ptr<lslidar_ls_driver::srv::AngleDistortionCorrection::Request> req,
                                                     std::shared_ptr<lslidar_ls_driver::srv::AngleDistortionCorrection::Response> res) {
        if (!is_get_difop_) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        setPacketHeader(config_data);
        std::string correction  = "";
        if (req->angle_distortion_correction == 0) {
            config_data[45] = 0x00;
            correction  = "angle distortion correction off.";
        } else if (req->angle_distortion_correction == 1) {
            config_data[45] = 0x01;
            correction  = "angle distortion correction on.";
        } else {
            LS_ERROR << "Parameter error, please check the input parameters." << LS_END;
            res->result = false;
            return true;
        }

        res->result = true;
        sendPacketTolidar(config_data);

        LS_MSG << "------------------------------------------------------------" << LS_END;
        LS_MSG << "Set successfully! Current lidar " << correction << LS_END;
        LS_SOCKET << "Please restart the ROS2 driver." << LS_END;

        return true;
    }

    bool lslidarDriver::setDataIp(std::shared_ptr<lslidar_ls_driver::srv::DataIp::Request> req,
                                  std::shared_ptr<lslidar_ls_driver::srv::DataIp::Response> res) {
        std::regex ipv4(
                "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
        if (!regex_match(req->data_ip, ipv4)) {
            LS_ERROR << "Parameter error, please check the input parameters." << LS_END;
            res->result = false;
            return true;
        }

        if(!is_get_difop_) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!"<< LS_END;
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

        LS_MSG << "------------------------------------------------------------" << LS_END;
        LS_MSG <<"Set successfully! Current lidar ip:" << req->data_ip.c_str() << LS_END;
        LS_MSG <<"Please modify the corresponding parameters in the launch file" << LS_END;
        
        return true;
    }

    bool lslidarDriver::setDataPort(std::shared_ptr<lslidar_ls_driver::srv::DataPort::Request> req,
                                    std::shared_ptr<lslidar_ls_driver::srv::DataPort::Response> res) {
        if(!is_get_difop_) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
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

        LS_MSG << "------------------------------------------------------------" << LS_END;
        LS_MSG <<"Set successfully! Current lidar MSOP port:" << req->data_port << LS_END;
        LS_MSG <<"Please modify the corresponding parameters in the launch file" << LS_END;
        
        return true;
    }

    bool lslidarDriver::setDestinationIp(std::shared_ptr<lslidar_ls_driver::srv::DestinationIp::Request> req,
                                         std::shared_ptr<lslidar_ls_driver::srv::DestinationIp::Response> res) {
        std::regex ipv4(
                "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
        if (!regex_match(req->destination_ip, ipv4)) {
            LS_ERROR << "Parameter error, please check the input parameters" << LS_END;
            res->result = false;
            return true;
        }

        if(!is_get_difop_) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!"<< LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!"<< LS_END;
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

        LS_MSG << "------------------------------------------------------------" << LS_END;
        LS_MSG <<"Set successfully! Current lidar destination ip:" << req->destination_ip.c_str() << LS_END;
        LS_MSG <<"Please modify the local IP address" << LS_END;

        return true;
    }

    bool lslidarDriver::setDevPort(std::shared_ptr<lslidar_ls_driver::srv::DevPort::Request> req,
                                   std::shared_ptr<lslidar_ls_driver::srv::DevPort::Response> res) {
        if(!is_get_difop_) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
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

        LS_MSG << "------------------------------------------------------------" << LS_END;
        LS_MSG <<"Set successfully! Current lidar DIFOP port:" << req->dev_port << LS_END;
        LS_MSG <<"Please modify the corresponding parameters in the launch file" << LS_END;
        
        return true;
    }

    bool lslidarDriver::setFrameRate(std::shared_ptr<lslidar_ls_driver::srv::FrameRate::Request> req,
                                  std::shared_ptr<lslidar_ls_driver::srv::FrameRate::Response> res) {
        if (!is_get_difop_) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        setPacketHeader(config_data);
        std::string frame_rate_ = "";
        if (req->frame_rate == 0) {
            config_data[100] = 0x00;
            frame_rate_ = "Standard frame rate";
        } else if (req->frame_rate == 25) {
            config_data[100] = 0x02;
            frame_rate_ = "25 percent frame rate";
        } else if (req->frame_rate == 50) {
            config_data[100] = 0x01;
            frame_rate_ = "50 percent frame rate";
        } else {
            LS_ERROR << "Parameter error, please check the input parameters" << LS_END;
            res->result = false;
            return true;
        }

        res->result = true;
        sendPacketTolidar(config_data);

        LS_MSG << "------------------------------------------------------------" << LS_END;
        LS_MSG <<"Set successfully! Current lidar " << frame_rate_ << LS_END;
        
        return true;
    }

    bool lslidarDriver::setInvalidData(std::shared_ptr<lslidar_ls_driver::srv::InvalidData::Request> req,
                                       std::shared_ptr<lslidar_ls_driver::srv::InvalidData::Response> res) {
        if (!is_get_difop_) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        setPacketHeader(config_data);
        std::string invalid_data  = "";
        if (req->invalid_data == 0) {
            config_data[87] = 0x00;
            invalid_data  = "send invalid data.";
        } else if (req->invalid_data == 1) { // 不发送
            config_data[87] = 0xaa;
            invalid_data  = "do not send invalid data.";
        } else {
            LS_ERROR << "Parameter error, please check the input parameters." << LS_END;
            res->result = false;
            return true;
        }

        res->result = true;
        sendPacketTolidar(config_data);

        LS_MSG << "------------------------------------------------------------" << LS_END;
        LS_MSG <<"Set successfully!" << " Current lidar " << invalid_data  << LS_END;
        
        return true;
    }

    bool lslidarDriver::setStandbyMode(std::shared_ptr<lslidar_ls_driver::srv::StandbyMode::Request> req,
                                       std::shared_ptr<lslidar_ls_driver::srv::StandbyMode::Response> res) {
        if (!is_get_difop_) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        unsigned char config_data[1206];
        mempcpy(config_data, difop_data, 1206);
        if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
            res->result = 0;
            LS_WARN << "Can not get dev packet! Set failed!" << LS_END;
            return true;
        }

        setPacketHeader(config_data);
        std::string lidar_mode  = "";
        if (req->standby_mode == 0) {
            config_data[101] = 0x00;
            lidar_mode  = "is in normal mode.";
        } else if (req->standby_mode == 1) {     // 待机
            config_data[101] = 0x01;
            lidar_mode  = "is in standby mode.";
        } else {
            LS_ERROR << "Parameter error, please check the input parameters." << LS_END;
            res->result = false;
            return true;
        }

        res->result = true;
        sendPacketTolidar(config_data);

        LS_MSG << "------------------------------------------------------------" << LS_END;
        LS_MSG <<"Set successfully! Current lidar " << lidar_mode  << LS_END;
        
        return true;
    }

    bool lslidarDriver::setTimeService(std::shared_ptr<lslidar_ls_driver::srv::TimeService::Request> req,
                                       std::shared_ptr<lslidar_ls_driver::srv::TimeService::Response> res) {
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
            config_data[44] = 0x00;
        } else if (time_service_mode == "ptp_l2") {
            config_data[44] = 0x01;
        } else if (time_service_mode == "ptp_udpv4") {
            config_data[44] = 0x03;
        } else if (time_service_mode == "ntp") {
            config_data[44] = 0x02;
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

        LS_MSG << "------------------------------------------------------------" << LS_END;
        LS_MSG << "The current timing method is " << time_service_mode << ". Time service method modified successfully!" << LS_END;;
        
        return true;
    }
}  // namespace lslidar_driver

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


namespace lslidar_driver {
    using namespace std::chrono;

    lslidarDriver::lslidarDriver() : lslidarDriver(rclcpp::NodeOptions()) {}

    lslidarDriver::lslidarDriver(const rclcpp::NodeOptions &options) : Node("lslidar_node", options),
                                                                       socket_id(-1),
                                                                       time_synchronization(false),
                                                                       min_range(0.15),
                                                                       max_range(200),
                                                                       packet_end_time(0.0),
                                                                       current_packet_time(0.0),
                                                                       last_packet_time(0.0),
                                                                       return_mode(1),
                                                                       point_time(0.0),
                                                                       g_fAngleAcc_V(0.01),
                                                                       is_add_frame_(false),
                                                                       is_get_difop_(false),
                                                                       last_packet_number_(0),
                                                                       current_packet_number_(0), total_packet_loss_(0),
                                                                       frame_count(0),
                                                                       threadPool_(std::make_unique<ThreadPool>(2)),
                                                                       point_cloud_xyzirt_(new pcl::PointCloud<VPoint>),
                                                                       point_cloud_xyzirt_bak_(
                                                                               new pcl::PointCloud<VPoint>),
                                                                       point_cloud_xyzirt_pub_(
                                                                               new pcl::PointCloud<VPoint>) {
        // create the sin and cos table for different azimuth and vertical values
        for (int j = 0; j < 36000; ++j) {
            double angle = static_cast<double>(j) / 100.0 * M_PI / 180.0;
            sin_table[j] = sin(angle);
            cos_table[j] = cos(angle);
        }

        double mirror_angle[8] = {-2.555, -1.825, -1.095, -0.365, 0.365, 1.095, 1.825, 2.555};   //摆镜角度   //根据通道不同偏移角度不同
        for (int i = 0; i < 8; ++i) {
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
        this->declare_parameter<std::string>("frame_id", "laser_link");
        this->declare_parameter<bool>("is_add_frame_", false);
        this->declare_parameter<std::string>("device_ip", "192.168.1.200");
        this->declare_parameter<bool>("add_multicast", false);
        this->declare_parameter<std::string>("group_ip", "234.2.3.2");
        this->declare_parameter<int>("msop_port", 2368);
        this->declare_parameter<int>("difop_port", 2369);
        this->declare_parameter<int>("scan_num", 8);
        this->declare_parameter<bool>("time_synchronization", false);
        this->declare_parameter<std::string>("pcap", "");
        this->declare_parameter<double>("min_range", 0.3);
        this->declare_parameter<double>("max_range", 150.0);
        this->declare_parameter<std::string>("topic_name", "lslidar_point_cloud");
        this->declare_parameter<bool>("publish_scan", false);
        this->declare_parameter<int>("scan_start_angle", -60);
        this->declare_parameter<int>("scan_end_angle", 60);
        this->declare_parameter<double>("packet_rate", 29815.0);
        this->declare_parameter<double>("Vertical_angle_compensation", -28.0);

        msop_udp_port = 0;
        difop_udp_port = 0;
        this->get_parameter("pcap", dump_file);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("is_add_frame_", is_add_frame_);
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
        this->get_parameter("packet_rate", packet_rate);
        this->get_parameter("Vertical_angle_compensation", Angle_V_compensation);
        
        RCLCPP_INFO(this->get_logger(), "dump_file: %s", dump_file.c_str());
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "is_add_frame_: %d", is_add_frame_);
        RCLCPP_INFO(this->get_logger(), "group_ip_string: %s", group_ip_string.c_str());
        RCLCPP_INFO(this->get_logger(), "add_multicast: %d", add_multicast);
        RCLCPP_INFO(this->get_logger(), "device_ip: %s", lidar_ip_string.c_str());
        RCLCPP_INFO(this->get_logger(), "msop_port: %d", msop_udp_port);
        RCLCPP_INFO(this->get_logger(), "difop_udp_port: %d", difop_udp_port);
        RCLCPP_INFO(this->get_logger(), "min_range: %f", min_range);
        RCLCPP_INFO(this->get_logger(), "max_range: %f", max_range);
        RCLCPP_INFO(this->get_logger(), "scan_start_angle: %d", scan_start_angle);
        RCLCPP_INFO(this->get_logger(), "scan_end_angle: %d", scan_end_angle);
        RCLCPP_INFO(this->get_logger(), "time_synchronization: %d", time_synchronization);
        RCLCPP_INFO(this->get_logger(), "topic_name: %s", pointcloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "packet_rate: %f", packet_rate);
        RCLCPP_INFO(this->get_logger(),"Vertical_angle_compensation: %f", Angle_V_compensation);

        inet_aton(lidar_ip_string.c_str(), &lidar_ip);
        if (add_multicast)
            RCLCPP_INFO(this->get_logger(), "opening UDP socket: group_address %s", group_ip_string.c_str());
        return true;
    }

    bool lslidarDriver::createRosIO() {
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
        packet_loss_pub_ = this->create_publisher<std_msgs::msg::Int32>("packet_loss", 10);

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

    bool lslidarDriver::initialize() {
        this->initTimeStamp();
        if (!loadParameters()) {
            RCLCPP_DEBUG(this->get_logger(), "Cannot load all required ROS parameters...");
            return false;
        }

        if (!createRosIO()) {
            RCLCPP_DEBUG(this->get_logger(), "Cannot create all ROS IO...");
            return false;
        }
        return true;
    }

    bool lslidarDriver::polling() {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet(new lslidar_msgs::msg::LslidarPacket());

        // Since the lslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            // keep reading until full packet received
            int rc = msop_input_->getPacket(packet);
            if (rc == 0) {
                break;       // got a full packet?
            }
            if (rc < 0) return false; // end of file reached?
        }

        // publish message using time of last packet read
        RCLCPP_DEBUG(this->get_logger(), "Publishing a full lslidar scan");
        if (time_synchronization) {
            // it is already the msop msg
            // use the first packets
            lslidar_msgs::msg::LslidarPacket pkt = *packet;
            if (0xff == pkt.data[1194]) {    //ptp授时
                //std::cout << "ptp";
                uint64_t timestamp_s = (pkt.data[1195] * 0 + (pkt.data[1196] << 24) + (pkt.data[1197] << 16) +
                                        (pkt.data[1198] << 8) + pkt.data[1199]);
                uint64_t timestamp_nsce =
                        (pkt.data[1200] << 24) + (pkt.data[1201] << 16) + (pkt.data[1202] << 8) + (pkt.data[1203]);
                timeStamp = rclcpp::Time(timestamp_s, timestamp_nsce);  // s,ns
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
                packet_timestamp =
                        pkt.data[1203] + (pkt.data[1202] << 8) + (pkt.data[1201] << 16) + (pkt.data[1200] << 24); //ns

                timeStamp = rclcpp::Time(pointcloudTimeStamp, packet_timestamp);  // s,ns
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
            }
        } else {
            packet->stamp = get_clock()->now();
            current_packet_time = rclcpp::Time(packet->stamp).seconds();
        }
        lslidarChPacketProcess(packet);

        return true;
    }

    void lslidarDriver::initTimeStamp() {
        for (unsigned char &i : this->packetTimeStamp) {
            i = 0;
        }
        this->pointcloudTimeStamp = 0;
        this->timeStamp = rclcpp::Time(0.0);
    }

    void lslidarDriver::difopPoll() {
        lslidar_msgs::msg::LslidarPacket::UniquePtr difop_packet(new lslidar_msgs::msg::LslidarPacket());
        // reading and publishing scans as fast as possible.
        while (rclcpp::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc == 0) {
                if (difop_packet->data[0] == 0x00 || difop_packet->data[0] == 0xa5) {
                    if (difop_packet->data[1] == 0xff && difop_packet->data[2] == 0x00 &&
                        difop_packet->data[3] == 0x5a) {
                        
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
        pc_msg.header.stamp = packet_timeStamp;
        pointcloud_pub_->publish(pc_msg);
//        RCLCPP_INFO(this->get_logger(), "pointcloud size: %u", pc_msg.width);
    }

    int lslidarDriver::convertCoordinate(const struct Firing &lidardata) {

        double fAngle_H = lidardata.azimuth;           //水平角度
        double fAngle_V = lidardata.vertical_angle;    // 垂直角度

        double Angle_H = fAngle_H;           //水平角度
        double Angle_V = fAngle_V + Angle_V_compensation;    // 垂直角度

        while (Angle_V < 0.0) {
            Angle_V += 360.0;
        }
        while (Angle_H < 0.0) {
            Angle_H += 360.0;
        }
        
        int table_index_V = int(Angle_V * 100) % 36000;
        int table_index_H = int(Angle_H * 100) % 36000;

        double fAngle_R0 = cos_mirror_angle[lidardata.channel_number % 8] * cos_table[table_index_H] * cos135 -
                           sin135 * sin_mirror_angle[lidardata.channel_number % 8];

        double fSinH_angle = 2 * fAngle_R0 * sin135 + sin_mirror_angle[lidardata.channel_number % 8];
        double fCosH_angle = sqrt(1 - pow(fSinH_angle, 2));

        double cosH_angle = (2 * fAngle_R0 * cos135 - cos_table[table_index_H] * cos_mirror_angle[lidardata.channel_number % 8]) / fCosH_angle;

        double fAngle_R1 = fCosH_angle * cosH_angle * cos_table[table_index_V]
                - sin_table[table_index_V] * fSinH_angle;

        double fSin_angle_V = 2 * fAngle_R1 * sin_table[table_index_V] + fSinH_angle;
        double fcos_angle_V = sqrt(1 - pow(fSin_angle_V, 2));

        double Angle_V_out = asin(fSin_angle_V) * 180 / M_PI;

        double fcos_angle_H = (2 * fAngle_R1 * cos_table[table_index_V] -
                cosH_angle * fCosH_angle) / fcos_angle_V;

        if (fcos_angle_H > 1)
        {
            fcos_angle_H = 1;
        }
        else if (fcos_angle_H < -1)
        {
            fcos_angle_H = -1;
        }

        double fSin_angle_H = sqrt(1 - pow(fcos_angle_H, 2));

        double Angle_H_out = acos(fcos_angle_H) * 180 / M_PI;

        if (fAngle_H < 0)
        {
            Angle_H_out = Angle_H_out - 180;
            fSin_angle_H *= -1.0;
        }
        else
        {
            Angle_H_out = 180 - Angle_H_out;
        }

        fAngle_H = Angle_H_out;
        fAngle_V = -Angle_V_out;

        double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;
        x_coord = (lidardata.distance * fcos_angle_V * fSin_angle_H) * g_fDistanceAcc;
        y_coord = (-1 * lidardata.distance * fcos_angle_V * fcos_angle_H) * g_fDistanceAcc;
        z_coord = (-1 * lidardata.distance * fSin_angle_V) * g_fDistanceAcc;

        //pcl::PointXYZI point;
        VPoint point;
        point.x = x_coord;
        point.y = y_coord;
        point.z = z_coord;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.channel_number;
        point.timestamp = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        point_cloud_xyzirt_bak_->points.push_back(point);
        ++point_cloud_xyzirt_bak_->width;
        return 0;
    }

    void lslidarDriver::lslidarChPacketProcess(const lslidar_msgs::msg::LslidarPacket::UniquePtr &msg) {
        struct Firing lidardata{};
        // Convert the msg to the raw packet type.

        packet_timeStamp = msg->stamp;
        packet_end_time = packet_timeStamp.seconds();
        bool packetType = false;
        if (msg->data[1205] == 0x02) {
            return_mode = 2;
        }

        if (return_mode == 1) {
            #ifdef PACKETS_LOSS
            current_packet_number_ = (msg->data[1192] << 8) + msg->data[1193];
            if ((current_packet_number_ - last_packet_number_ > 1 &&
                 last_packet_number_ - current_packet_number_ != 65535) && last_packet_number_ != 0) {
                total_packet_loss_ += current_packet_number_ - last_packet_number_ - 1;
                RCLCPP_DEBUG(this->get_logger(), "packet loss = %lu", total_packet_loss_);
                std_msgs::msg::Int32 loss_data;
                loss_data.data = total_packet_loss_;
                packet_loss_pub_->publish(loss_data);
            }
            last_packet_number_ = current_packet_number_;
            #endif

            double packet_interval_time =
                    (current_packet_time - last_packet_time) / (POINTS_PER_PACKET_SINGLE_ECHO / 8.0);
            for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_SINGLE_ECHO; point_idx += 8) {
                if ((msg->data[point_idx] == 0xff) && (msg->data[point_idx + 1] == 0xaa) &&
                    (msg->data[point_idx + 2] == 0xbb) && (msg->data[point_idx + 3] == 0xcc) &&
                    (msg->data[point_idx + 4] == 0xdd)) {
                    packetType = true;
                    frame_count++;
                } else {
                    // Compute the time of the point
                    if (last_packet_time > 1e-6) {
                        point_time = packet_end_time -
                                     packet_interval_time * ((POINTS_PER_PACKET_SINGLE_ECHO - point_idx) / 8 - 1);
                        point_time = point_time < 0.0 ? 0.0 : point_time;
                    } else {
                        point_time = current_packet_time;
                    }

                    memset(&lidardata, 0, sizeof(lidardata));
                    //水平角度
                    double fAngle_H = msg->data[point_idx + 1] + (msg->data[point_idx] << 8);
                    if (fAngle_H > 32767) {
                        fAngle_H = (fAngle_H - 65536);
                    }
                    lidardata.azimuth = fAngle_H * 0.01;     // 一包中每个点的水平角度
                    if ((lidardata.azimuth < scan_start_angle) || (lidardata.azimuth > scan_end_angle)) continue;

                    lidardata.distance = ((msg->data[point_idx + 4] << 16) + (msg->data[point_idx + 5] << 8) +
                                          msg->data[point_idx + 6]);
                    if (!isPointInRange(lidardata.distance * g_fDistanceAcc)) continue;
                    //垂直角度+通道号
                    int iTempAngle = msg->data[point_idx + 2];
                    int iChannelNumber = iTempAngle >> 5;    //左移5位 通道号
                    int iSymmbol = (iTempAngle >> 4) & 0x01; //左移4位 符号位
                    double fAngle_V = 0.0f;
                    if (1 == iSymmbol) // 符号位 0：正数 1：负数
                    {
                        int iAngle_V = msg->data[point_idx + 3] + (msg->data[point_idx + 2] << 8);

                        fAngle_V = iAngle_V | 0xE000;
                        if (fAngle_V > 32767) {
                            fAngle_V = (fAngle_V - 65536);
                        }
                    } else {
                        int iAngle_Hight = iTempAngle & 0x1f;
                        fAngle_V = msg->data[point_idx + 3] + (iAngle_Hight << 8);
                    }

                    lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
                    lidardata.channel_number = iChannelNumber;
                    lidardata.intensity = msg->data[point_idx + 7];
                    lidardata.time = point_time;

                    convertCoordinate(lidardata);
                }

                if (packetType) {
                    if (is_add_frame_) {
                        if (frame_count >= 2) {
                            {
                                std::unique_lock<std::mutex> lock(pc_lock);
                                point_cloud_xyzirt_pub_ = point_cloud_xyzirt_;
                            }
                            threadPool_->enqueue([&]() { publishPointCloud(); });
                        }
                        packetType = false;
                        point_cloud_xyzirt_ = point_cloud_xyzirt_bak_;
                        point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>);
                    } else {
                        {
                            std::unique_lock<std::mutex> lock(pc_lock);
                            point_cloud_xyzirt_pub_ = point_cloud_xyzirt_;
                        }
                        threadPool_->enqueue([&]() { publishPointCloud(); });
                        packetType = false;
                        point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                        point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>);
                    }
                }
            }
        } else {
            #ifdef PACKETS_LOSS
            current_packet_number_ =
                    msg->data[1188] * 0 + (msg->data[1189] * 4294967296) + (msg->data[1190] << 24) + (msg->data[1191] << 16) +
                    (msg->data[1192] << 8) + msg->data[1193];
            if (current_packet_number_ - last_packet_number_ > 1 && last_packet_number_ != 0) {
                total_packet_loss_ += current_packet_number_ - last_packet_number_ - 1;
                RCLCPP_DEBUG(this->get_logger(), "packet loss = %lu", total_packet_loss_);
                std_msgs::msg::Int32 loss_data;
                loss_data.data = total_packet_loss_;
                packet_loss_pub_->publish(loss_data);
            }
            last_packet_number_ = current_packet_number_;
            #endif
            double packet_interval_time = (current_packet_time - last_packet_time) / (POINTS_PER_PACKET_DOUBLE_ECHO / 12.0);
            for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_DOUBLE_ECHO; point_idx += 12) {
                if ((msg->data[point_idx] == 0xff) && (msg->data[point_idx + 1] == 0xaa) &&
                    (msg->data[point_idx + 2] == 0xbb) && (msg->data[point_idx + 3] == 0xcc) &&
                    (msg->data[point_idx + 4] == 0xdd)) {
                    packetType = true;
                    frame_count++;
                } else {
                    // Compute the time of the point
                    if (last_packet_time > 1e-6) {
                        point_time =
                                packet_end_time - packet_interval_time * ((POINTS_PER_PACKET_DOUBLE_ECHO - point_idx) / 12 - 1);
                    } else {
                        point_time = current_packet_time;
                    }
                    memset(&lidardata,0, sizeof(lidardata));
                    //水平角度
                    double fAngle_H = msg->data[point_idx + 1] + (msg->data[point_idx] << 8);
                    if (fAngle_H > 32767) {
                        fAngle_H = (fAngle_H - 65536);
                    }
                    lidardata.azimuth = fAngle_H * 0.01f;
                    if ((lidardata.azimuth < scan_start_angle) || (lidardata.azimuth > scan_end_angle)) continue;

                    //垂直角度+通道号
                    int iTempAngle = msg->data[point_idx + 2];
                    int iChannelNumber = iTempAngle >> 5;     //左移5位 通道号
                    int iSymmbol = (iTempAngle >> 4) & 0x01;  //左移4位 符号位
                    double fAngle_V = 0.0f;
                    if (1 == iSymmbol) // 符号位 0：正数 1：负数
                    {
                        int iAngle_V = msg->data[point_idx + 3] + (msg->data[point_idx + 2] << 8);

                        fAngle_V = iAngle_V | 0xE000;
                        if (fAngle_V > 32767) {
                            fAngle_V = (fAngle_V - 65536);
                        }
                    } else {
                        int iAngle_Hight = iTempAngle & 0x1f;
                        fAngle_V = msg->data[point_idx + 3] + (iAngle_Hight << 8);
                    }

                    lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
                    lidardata.channel_number = iChannelNumber;
                    lidardata.distance = ((msg->data[point_idx + 4] << 16) + (msg->data[point_idx + 5] << 8) +
                                          msg->data[point_idx + 6]);
                    if (!isPointInRange(lidardata.distance * g_fDistanceAcc)) continue;
                    lidardata.intensity = msg->data[point_idx + 7];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);  // 第一个点

                    lidardata.distance = ((msg->data[point_idx + 8] << 16) + (msg->data[point_idx + 9] << 8) +
                                          msg->data[point_idx + 10]);
                    if (!isPointInRange(lidardata.distance * g_fDistanceAcc)) continue;
                    lidardata.intensity = msg->data[point_idx + 11];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);  // 第二个点
                }

                if (packetType) {
                    if (is_add_frame_) {
                        if (frame_count >= 2) {
                            {
                                std::unique_lock<std::mutex> lock(pc_lock);
                                point_cloud_xyzirt_pub_ = point_cloud_xyzirt_;
                            }
                            threadPool_->enqueue([&]() { publishPointCloud(); });
                        }
                        packetType = false;
                        point_cloud_xyzirt_ = point_cloud_xyzirt_bak_;
                        point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>);
                    } else {
                        {
                            std::unique_lock<std::mutex> lock(pc_lock);
                            point_cloud_xyzirt_pub_ = point_cloud_xyzirt_;
                        }
                        threadPool_->enqueue([&]() { publishPointCloud(); });
                        packetType = false;
                        point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                        point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>);
                    }
                }
            }
        }
        last_packet_time = current_packet_time;
    }

}  // namespace lslidar_driver

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


#ifndef LSLIDAR_SERVICES_HPP
#define LSLIDAR_SERVICES_HPP

#include <atomic>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>  
#include <regex>
#include <cstdio>
#include <string>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "lslidar_log.hpp"
#include "lslidar_msgs/msg/lslidar_packet.hpp"
#include "lslidar_msgs/srv/angle_distortion_correction.hpp"
#include "lslidar_msgs/srv/frame_rate.hpp"
#include "lslidar_msgs/srv/invalid_data.hpp"
#include "lslidar_msgs/srv/ip_and_port.hpp"
#include "lslidar_msgs/srv/motor_control.hpp"
#include "lslidar_msgs/srv/motor_speed.hpp"
#include "lslidar_msgs/srv/power_control.hpp"
#include "lslidar_msgs/srv/rfd_removal.hpp"
#include "lslidar_msgs/srv/standby_mode.hpp"
#include "lslidar_msgs/srv/tail_removal.hpp"
#include "lslidar_msgs/srv/time_mode.hpp"

// DIFOP 设备包
// UCWP  配置包

namespace lslidar_driver {

    struct NetConfig {
        std::string lidar_ip;
        std::string destination_ip;
        uint16_t data_port;
        uint16_t dev_port;
    };

    class LslidarServices {
    public:
        LslidarServices() {}

        virtual ~LslidarServices() = default;

        // 获取设备包数据
        int getDifopPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &pkt);

        // 设置配置包包头
        static void setUcwpPacketHeader(unsigned char *ucwp_data);

        // 设置配置包
        bool setUcwpData(unsigned char *ucwp_data);

        // 发送配置包配置雷达
        bool sendUcwpPacketTolidar(unsigned char *ucwp_data) const;

        virtual void setTimeModeBytes(unsigned char* ucwp_data, int time_mode);

        std::string getTimeModeString(int time_mode);

        bool checkAndSetIp(const std::string& ip, unsigned char* ucwp_data, int index, bool is_lidar_ip);

        bool checkAndSetPort(int port, unsigned char* ucwp_data, int index, bool is_data_port);

        bool loadConfigFromYAML();

        // 设置雷达IP 端口
        // 支持 手动传参与配置文件传参
        // 出错保护: 禁止将雷达ip与当前雷达目的ip设置相同，数据端口与当前设备端口设置相同，反之亦然
        virtual bool setIpAndPort(std::shared_ptr<lslidar_msgs::srv::IpAndPort::Request> req,
                                  std::shared_ptr<lslidar_msgs::srv::IpAndPort::Response> res);

        // 设置雷达转速
        virtual bool setMotorSpeed(std::shared_ptr<lslidar_msgs::srv::MotorSpeed::Request> req,
                                   std::shared_ptr<lslidar_msgs::srv::MotorSpeed::Response> res);

        // 设置雷达授时模式
        virtual bool setTimeMode(std::shared_ptr<lslidar_msgs::srv::TimeMode::Request> req,
                                 std::shared_ptr<lslidar_msgs::srv::TimeMode::Response> res);

    protected:
        std::mutex difop_data_mutex;
        std::atomic<bool> difop_valid{false};
        unsigned char difop_data[1206];
        std::string lslidar_ip;

        NetConfig LiDAR;
    };


    class LslidarCxServices : public LslidarServices {
    public:
        LslidarCxServices() {}

        // 机械式雷达设置 IP 端口
        bool setIpAndPort(std::shared_ptr<lslidar_msgs::srv::IpAndPort::Request> req,
                          std::shared_ptr<lslidar_msgs::srv::IpAndPort::Response> res) override;

        // 机械式雷达设置转速
        bool setMotorSpeed(std::shared_ptr<lslidar_msgs::srv::MotorSpeed::Request> req,
                           std::shared_ptr<lslidar_msgs::srv::MotorSpeed::Response> res) override;
        
        // 机械式雷达控制电机启停
        bool setMotorControl(std::shared_ptr<lslidar_msgs::srv::MotorControl::Request> req,
                             std::shared_ptr<lslidar_msgs::srv::MotorControl::Response> res);
        
        // 机械式雷达控制雷达上下电
        bool setPowerControl(std::shared_ptr<lslidar_msgs::srv::PowerControl::Request> req,
                             std::shared_ptr<lslidar_msgs::srv::PowerControl::Response> res);
        
        // 机械式雷达去雨雾尘
        bool setRfdRemoval(std::shared_ptr<lslidar_msgs::srv::RfdRemoval::Request> req,
                           std::shared_ptr<lslidar_msgs::srv::RfdRemoval::Response> res);
        
        // 机械式雷达设置去除拖尾
        bool setTailRemoval(std::shared_ptr<lslidar_msgs::srv::TailRemoval::Request> req,
                            std::shared_ptr<lslidar_msgs::srv::TailRemoval::Response> res);

        // 机械式雷达设置授时模式
        bool setTimeMode(std::shared_ptr<lslidar_msgs::srv::TimeMode::Request> req,
                         std::shared_ptr<lslidar_msgs::srv::TimeMode::Response> res) override;
        
        void setTimeModeBytes(unsigned char* ucwp_data, int time_mode) override;

        int GetCxFpgaVersion(unsigned char* ucwp_data);
    };


    /////////////////////// 905 ///////////////////////
    class LslidarChServices : public LslidarServices {
    public:
        LslidarChServices() {}

        // 905雷达设置 IP 端口
        bool setIpAndPort(std::shared_ptr<lslidar_msgs::srv::IpAndPort::Request> req,
                          std::shared_ptr<lslidar_msgs::srv::IpAndPort::Response> res) override;

        // 905雷达设置转速
        bool setMotorSpeed(std::shared_ptr<lslidar_msgs::srv::MotorSpeed::Request> req,
                           std::shared_ptr<lslidar_msgs::srv::MotorSpeed::Response> res) override;

        bool setTimeMode(std::shared_ptr<lslidar_msgs::srv::TimeMode::Request> req,
                         std::shared_ptr<lslidar_msgs::srv::TimeMode::Response> res) override;
    };



    ////////////////////// 1550 //////////////////////
    class LslidarLsServices : public LslidarServices {
    public:
        LslidarLsServices() {}

        // 1550雷达设置角度畸变矫正
        bool setAngleDistortionCorrection(std::shared_ptr<lslidar_msgs::srv::AngleDistortionCorrection::Request> req,
                                          std::shared_ptr<lslidar_msgs::srv::AngleDistortionCorrection::Response> res);

        // 1550雷达设置 IP 端口
        bool setIpAndPort(std::shared_ptr<lslidar_msgs::srv::IpAndPort::Request> req,
                          std::shared_ptr<lslidar_msgs::srv::IpAndPort::Response> res) override;
        
        bool setTimeMode(std::shared_ptr<lslidar_msgs::srv::TimeMode::Request> req,
                         std::shared_ptr<lslidar_msgs::srv::TimeMode::Response> res) override;

        // 1550雷达设置帧率
        bool setFrameRate(std::shared_ptr<lslidar_msgs::srv::FrameRate::Request> req,
                          std::shared_ptr<lslidar_msgs::srv::FrameRate::Response> res);

        // 1550雷达设置是否发布无效数据
        bool setInvalidData(std::shared_ptr<lslidar_msgs::srv::InvalidData::Request> req,
                            std::shared_ptr<lslidar_msgs::srv::InvalidData::Response> res);

        // 1550雷达设置待机模式
        bool setStandbyMode(std::shared_ptr<lslidar_msgs::srv::StandbyMode::Request> req,
                            std::shared_ptr<lslidar_msgs::srv::StandbyMode::Response> res);
    };
}

#endif // LSLIDAR_SERVICES_HPP

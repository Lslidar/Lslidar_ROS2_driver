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

#ifndef LSLIDAR_DEVICE_INFO_HPP
#define LSLIDAR_DEVICE_INFO_HPP

#include <string>               
#include <sstream>         
#include <iomanip> 
#include <vector>           
#include <algorithm>   
#include <cctype>               
#include <memory>              
#include <iostream> 
#include "lslidar_log.hpp"  
#include <rclcpp/rclcpp.hpp> 
#include "lslidar_msgs/msg/lslidar_packet.hpp" 
#include "lslidar_msgs/msg/lslidar_information.hpp" 

namespace lslidar_driver {

    struct WorkingTime {
        int total_working_time_pos[4];
        int less_minus40_pos[3];       // < -40℃
        int minus40_to_minus10_pos[3]; // -40℃ ~ -10℃ 
        int minus10_to_30_pos[3];      // -10℃ ~ 30℃ 
        int positive30_to_70_pos[3];   // 30℃ ~ 70℃ 
        int positive70_to_100_pos[3];  // 70℃ ~ 100℃ 
    };

    struct Information {
        std::string lidarIp;
        std::string destinationIP;
        std::string lidarMacAddress;
        uint16_t msopPort;
        uint16_t difopPort;
        std::string lidarSerialNumber;
        std::string secondBoardProgram;
        std::string thirdBoardProgram;
    };

    class LidarDeviceInfo {
    public:
        LidarDeviceInfo() {}

        ~LidarDeviceInfo() = default;

        // 获取 机械式 雷达运行时间
        void parseAndPrintWorkingTime(const uint8_t* data, const WorkingTime& mapping);

        // 获取 雷达序列号
        std::string extractSerialNumber(const unsigned char* data, int length);

        // 获取 机械式 雷达型号
        std::string getCxLidarType(uint8_t data);

        // 获取 机械式 雷达设备信息
        Information getCxDeviceInfo(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet, int fpga_type);
        
        // 获取 905 1550 雷达设备信息
        Information getDeviceInfo(const lslidar_msgs::msg::LslidarPacket::UniquePtr &packet);

        std::string hex_to_string(unsigned int hexValue);

    private:
    
    };

}

#endif  // LSLIDAR_DEVICE_INFO_HPP


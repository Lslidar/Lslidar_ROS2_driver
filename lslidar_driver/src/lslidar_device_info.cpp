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

#include "lslidar_driver/lslidar_device_info.hpp"

namespace lslidar_driver {

    void LidarDeviceInfo::parseAndPrintWorkingTime(const uint8_t* data, const WorkingTime& mapping) {
        int total_working_time = (data[mapping.total_working_time_pos[0]] << 24) +
                                 (data[mapping.total_working_time_pos[1]] << 16) +
                                 (data[mapping.total_working_time_pos[2]] << 8) +
                                  data[mapping.total_working_time_pos[3]];

        auto parse3Bytes = [&](const int* pos) {
            return (data[pos[0]] << 16) + (data[pos[1]] << 8) + data[pos[2]];
        };

        int less_minus40_degree_working_time = parse3Bytes(mapping.less_minus40_pos);
        int minus40_to_minus10_working_time = parse3Bytes(mapping.minus40_to_minus10_pos);
        int minus10_to_30_working_time = parse3Bytes(mapping.minus10_to_30_pos);
        int positive30_to_70_working_time = parse3Bytes(mapping.positive30_to_70_pos);
        int positive70_to_100_working_time = parse3Bytes(mapping.positive70_to_100_pos);

        LS_INFO << "total working time: " << total_working_time / 60 << " hours:" << total_working_time % 60 << " minutes" << LS_END;
        LS_INFO << "less than -40 degrees working time: " << less_minus40_degree_working_time / 60 << " hours:" << less_minus40_degree_working_time % 60 << " minutes" << LS_END;
        LS_INFO << "-40 degrees ~ -10 degrees working time: " << minus40_to_minus10_working_time / 60 << " hours:" << minus40_to_minus10_working_time % 60 << " minutes" << LS_END;
        LS_INFO << "-10 degrees ~ 30 degrees working time: " << minus10_to_30_working_time / 60 << " hours:" << minus10_to_30_working_time % 60 << " minutes" << LS_END;
        LS_INFO << "30 degrees ~ 70 degrees working time: " << positive30_to_70_working_time / 60 << " hours:" << positive30_to_70_working_time % 60 << " minutes" << LS_END;
        LS_INFO << "70 degrees ~ 100 degrees working time: " << positive70_to_100_working_time / 60 << " hours:" << positive70_to_100_working_time % 60 << " minutes" << LS_END;
    }

    std::string LidarDeviceInfo::extractSerialNumber(const unsigned char* data, int length) {
        std::string serialNumber;
        for (int i = 0; i < length; i++) {
            char ch = static_cast<char>(data[i]);
            if (isprint(static_cast<unsigned char>(ch))) {
                serialNumber += ch;
            }
        }

        serialNumber.erase(std::find_if(serialNumber.rbegin(), serialNumber.rend(), [](unsigned char ch) {
            return ch != '\0';
        }).base(), serialNumber.end());

        serialNumber.erase(serialNumber.find_last_not_of(' ') + 1);

        size_t endpos = serialNumber.find_last_not_of(' ');
        if (endpos != std::string::npos) {
            serialNumber.erase(endpos + 1);
        } else {
            serialNumber.clear();
        }

        return serialNumber;
    }

    std::string LidarDeviceInfo::getCxLidarType(uint8_t data) {
        switch (data) {
            case 0: return "C32 ";
            case 1: return "C16 ";
            case 2: return "C8 ";
            case 3: return "C1 ";
            case 4: return "C32W ";
            case 5: return "CH32R ";
            case 6: return "N301 ";
            case 7: return "MS-C16 ";
            case 100: return "C32 ";    // 5.0
            case 101: return "C16 ";    // 5.0
            case 200: return "MS32L ";  // 5.0
            default: return "Unknown ";
        }
    }

    std::string LidarDeviceInfo::hex_to_string(unsigned int hexValue) {
        std::ostringstream oss;
        oss << std::hex << std::setw(2) << std::setfill('0') << hexValue;
        return oss.str();
    }

    Information LidarDeviceInfo::getCxDeviceInfo(const lslidar_msgs::msg::LslidarPacket::UniquePtr& packet, int fpga_type) {
        Information device{};
        std::string lidarType2, lidarType3, firmwareDate2, firmwareDate3, version2, version3;
        // 提取IP地址
        device.lidarIp = std::to_string(packet->data[10]) + "." + std::to_string(packet->data[11]) + "." +
                         std::to_string(packet->data[12]) + "." + std::to_string(packet->data[13]);

        device.destinationIP = std::to_string(packet->data[14]) + "." + std::to_string(packet->data[15]) + "." +
                               std::to_string(packet->data[16]) + "." + std::to_string(packet->data[17]);
        // 提取MAC地址
        std::ostringstream oss;
        for (int i = 18; i <= 23; ++i) {
            if (i != 18) {
                oss << ":";
            }
            oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(packet->data[i]);
        }
        device.lidarMacAddress = oss.str();

        // 端口号
        device.msopPort  = (packet->data[24] << 8) + packet->data[25];
        device.difopPort = (packet->data[26] << 8) + packet->data[27];

        if (fpga_type == 5) {
            device.lidarSerialNumber = extractSerialNumber(packet->data.data() + 1161, 20);

            lidarType2 = getCxLidarType(packet->data[1196]);
            lidarType3 = getCxLidarType(packet->data[1202]);

            firmwareDate2 = hex_to_string(packet->data[1191]) + 
                            hex_to_string(packet->data[1192]) + "-" + 
                            hex_to_string(packet->data[1193]) + "-" +  
                            hex_to_string(packet->data[1194]);
            firmwareDate3 = hex_to_string(packet->data[1197]) + 
                            hex_to_string(packet->data[1198]) + "-" + 
                            hex_to_string(packet->data[1199]) + "-" +  
                            hex_to_string(packet->data[1200]);

            version2 = " v" + std::to_string(packet->data[1203] >> 4) + "." +
                              std::to_string(packet->data[1203] & 0x0F)   + "." +
                              std::to_string(packet->data[1195]);
            version3 = " v" + std::to_string(packet->data[1203] >> 4) + "." +
                              std::to_string(packet->data[1203] & 0x0F)   + "." +
                              std::to_string(packet->data[1201]);
            
            device.secondBoardProgram = lidarType2 + firmwareDate2 + version2;
            device.thirdBoardProgram  = lidarType3 + firmwareDate3 + version3;
        } else if (fpga_type == 4) {
            // 序列号
            device.lidarSerialNumber = extractSerialNumber(packet->data.data() + 1166, 20);

            // FPGA版本信息
            lidarType2 = getCxLidarType(packet->data[1198] >> 4);
            lidarType3 = getCxLidarType(packet->data[1202] >> 4);
            
            firmwareDate2 = std::to_string((packet->data[1196] >> 4) + 2020 ) + "-" + 
                            std::to_string(packet->data[1196] & 0x0F) + "-" +  
                            std::to_string(packet->data[1197]);
            firmwareDate3 = std::to_string((packet->data[1200] >> 4) + 2020 ) + "-" + 
                            std::to_string(packet->data[1200] & 0x0F) + "-" +  
                            std::to_string(packet->data[1201]);

            version2 = " v" + std::to_string(packet->data[1198] & 0x0F) + "." +
                              std::to_string(packet->data[1199] >> 4)   + "." +
                              std::to_string(packet->data[1199] & 0x0F);
            version3 = " v" + std::to_string(packet->data[1202] & 0x0F) + "." +
                              std::to_string(packet->data[1203] >> 4)   + "." +
                              std::to_string(packet->data[1203] & 0x0F);

            device.secondBoardProgram = lidarType2 + firmwareDate2 + version2;
            device.thirdBoardProgram  = lidarType3 + firmwareDate3 + version3;
        } else {
            device.lidarSerialNumber = extractSerialNumber(packet->data.data() + 1164, 20);

            firmwareDate3 = std::to_string(packet->data[1198]) + 
                            std::to_string(packet->data[1199]) + "-" + 
                            std::to_string(packet->data[1200]) + "-" +  
                            std::to_string(packet->data[1201]);

            version2 = " v" + std::to_string(packet->data[1196]) + "." +
                              std::to_string(packet->data[1197] >> 4)   + "." +
                              std::to_string(packet->data[1197] & 0x0F);
            version3 = " v" + std::to_string(packet->data[1202]) + "." +
                              std::to_string(packet->data[1203] >> 4)   + "." +
                              std::to_string(packet->data[1203] & 0x0F);

            device.secondBoardProgram = version2;
            device.thirdBoardProgram  = firmwareDate3 + version3;
        }

        return device;
    }

    Information LidarDeviceInfo::getDeviceInfo(const lslidar_msgs::msg::LslidarPacket::UniquePtr& packet) {
        Information device{};
        std::string firmwareDate2, version2;
        // 提取IP地址
        device.lidarIp = std::to_string(packet->data[10]) + "." + std::to_string(packet->data[11]) + "." +
                         std::to_string(packet->data[12]) + "." + std::to_string(packet->data[13]);

        device.destinationIP = std::to_string(packet->data[14]) + "." + std::to_string(packet->data[15]) + "." +
                               std::to_string(packet->data[16]) + "." + std::to_string(packet->data[17]);
        // 提取MAC地址
        std::ostringstream oss;
        for (int i = 18; i <= 23; ++i) {
            if (i != 18) {
                oss << ":";
            }
            oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(packet->data[i]);
        }
        device.lidarMacAddress = oss.str();

        // 端口号
        device.msopPort  = (packet->data[24] << 8) + packet->data[25];
        device.difopPort = (packet->data[26] << 8) + packet->data[27];

        // 序列号
        device.lidarSerialNumber = extractSerialNumber(packet->data.data() + 1170, 20);

        firmwareDate2 = std::to_string((packet->data[1198] << 8) + packet->data[1199]) + "-" + 
                        std::to_string(packet->data[1200]) + "-" +  
                        std::to_string(packet->data[1201]);

        version2 = " v" + std::to_string(packet->data[1202]) + "." +
                          std::to_string(packet->data[1203] >> 4) + "." +
                          std::to_string(packet->data[1203] & 0x0F);
        
        device.secondBoardProgram =  firmwareDate2 + version2;

        return device;
    }
}

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

#ifndef LSIOSR_HPP
#define LSIOSR_HPP

#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <string>
#include <cerrno>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>

#include "lslidar_driver/lslidar_log.hpp"
#include "lslidar_msgs/msg/lslidar_packet.hpp"

namespace lslidar_driver {

// 波特率
enum class BaudRate {
    BAUD_230400 = 230400,
    BAUD_460800 = 460800,
    BAUD_500000 = 500000,
    BAUD_921600 = 921600
};

// 数据位
enum class DataBits {
    SEVEN = 7,
    EIGHT = 8
};

// 奇偶校验
enum class Parity {
    NONE,
    ODD,
    EVEN
};

// 停止位
enum class StopBits {
    ONE = 1,
    TWO = 2
};


class LSIOSR {
public:
    LSIOSR(const std::string& port, BaudRate baud_rate,
           DataBits data_bits = DataBits::EIGHT,
           Parity parity = Parity::NONE,
           StopBits stop_bits = StopBits::ONE);

    ~LSIOSR();

    LSIOSR(const LSIOSR&) = delete;

    LSIOSR& operator=(const LSIOSR&) = delete;

    void setOpt(DataBits nBits, Parity nEvent, StopBits nStop);

    ssize_t read(unsigned char* buffer, size_t length);

    ssize_t send(const char* buffer, size_t length = 188);

    int getSerialData(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet, int packet_length, std::string lidar_model);

    bool waitEvent(short events, int timeout_ms);

    void flushIO();

private:
    std::string port_;      // 串口名称
    BaudRate baud_rate_;    // 波特率
    DataBits data_bits_;    // 数据位
    Parity parity_;         // 奇偶校验
    StopBits stop_bits_;    // 停止位
    int fd_;                // 文件描述符
};

} // namespace lslidar_driver

#endif // LSIOSR_HPP

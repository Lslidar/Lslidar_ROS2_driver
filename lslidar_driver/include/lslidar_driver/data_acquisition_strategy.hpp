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

#ifndef DATA_ACQUISITION_STRATEGY_HPP
#define DATA_ACQUISITION_STRATEGY_HPP

#include <memory>
#include "lslidar_driver/input.hpp"
#include "lslidar_driver/lsiosr.hpp"
#include "lslidar_msgs/msg/lslidar_packet.hpp"

namespace lslidar_driver {

    class DataAcquisitionStrategy {
    public:
        virtual ~DataAcquisitionStrategy() = default;
        virtual int getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) = 0;
    };

    class NetworkStrategy : public DataAcquisitionStrategy {
    public:
        explicit NetworkStrategy(std::shared_ptr<lslidar_driver::Input> input)
            : msop_input_(std::move(input)) {}

        int getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) override {
            return msop_input_->getPacket(packet);
        }
    
    private:
        std::shared_ptr<Input> msop_input_;
    };

    class SerialStrategy : public DataAcquisitionStrategy {
    public:
        SerialStrategy(std::shared_ptr<lslidar_driver::LSIOSR> serial_input, int packet_length, const std::string& lidar_model)
            : serial_input_(std::move(serial_input)), packet_length_(packet_length), lidar_model_(lidar_model) {}

        int getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet) override {
            return serial_input_->getSerialData(packet, packet_length_, lidar_model_);
        }
        
    private:
        std::shared_ptr<LSIOSR> serial_input_;
        int packet_length_;
        std::string lidar_model_;
    };


} // namespace lslidar_driver

#endif // DATA_ACQUISITION_STRATEGY_HPP

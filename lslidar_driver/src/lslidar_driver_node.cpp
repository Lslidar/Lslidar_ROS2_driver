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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp> 
#include "lslidar_driver/lslidar_driver.hpp"
#include "lslidar_driver/lslidar_cx_driver.hpp"
#include "lslidar_driver/lslidar_ch_driver.hpp"
#include "lslidar_driver/lslidar_ls_driver.hpp"
#include "lslidar_driver/lslidar_x10_driver.hpp"

using namespace lslidar_driver;

volatile sig_atomic_t flag = 1;

static void my_handler(int sig) {
    flag = 0;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    auto node = std::make_shared<rclcpp::Node>("lslidar_driver_node", options);

    LS_PARAM << "************ Lslidar ROS driver version: " << lslidar_driver_VERSION << " ************" << LS_END;

    std::string lidar_type;
    node->declare_parameter<std::string>("lidar_type", "CX");
    node->get_parameter("lidar_type", lidar_type);

    std::shared_ptr<lslidar_driver::LslidarDriver> driver;

    try {
        if (lidar_type == "CX") {
            driver = std::make_shared<lslidar_driver::LslidarCxDriver>(node);
        } else if (lidar_type == "CH") {
            driver = std::make_shared<lslidar_driver::LslidarChDriver>(node);
        } else if (lidar_type == "LS") {
            driver = std::make_shared<lslidar_driver::LslidarLsDriver>(node);
        } else if (lidar_type == "X10") {
            driver = std::make_shared<lslidar_driver::LslidarX10Driver>(node);
        } else {
            LS_ERROR << "Invalid lidar type configured: '" << lidar_type 
                     << "'. Supported types are: CX, CH, LS, X10" << LS_END;
            throw std::invalid_argument("Unsupported lidar type");
        }

        if (!driver->initialize()) {
            LS_ERROR << "Failed to initialize the Lslidar driver." << LS_END;
            return -1;
        }
    } catch (const std::exception& e) {
        LS_ERROR << "Error: " << e.what() << LS_END;
        return -1;
    }

    std::unique_ptr<ThreadPool> threadPool = std::make_unique<ThreadPool>(1);
    threadPool->enqueue([&]() {
        while (rclcpp::ok()) {
            driver->poll();
        }
    });

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

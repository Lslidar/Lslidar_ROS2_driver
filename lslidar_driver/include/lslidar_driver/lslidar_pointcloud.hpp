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

#ifndef LSLIDAR_POINTCLOUD_HPP
#define LSLIDAR_POINTCLOUD_HPP

#include <pcl/point_types.h>  

/*********************************************************************************************
 * 点时间格式配置，必须与launch文件中 use_absolute_time 对应 
 * use_absolute_time为 true 时，取消 '#define USE_ABSOLUTE_TIME' 注释以使用绝对时间

 * Point time format configuration (must match 'use_absolute_time' in launch file)
 * When use_absolute_time=true, uncomment '#define USE_ABSOLUTE_TIME' to enable absolute timestamp format
 *********************************************************************************************/

// #define USE_ABSOLUTE_TIME

#ifdef USE_ABSOLUTE_TIME
    #define POINT_TIME_TYPE double  // 使用绝对时间 double
#else
    #define POINT_TIME_TYPE float   // 使用相对时间 float
#endif


namespace lslidar_driver {

    struct PointXYZIRT {
        PCL_ADD_POINT4D;      // x, y, z 和 data[4]
        PCL_ADD_INTENSITY;    // 强度
        std::uint16_t ring;   // 线号
        POINT_TIME_TYPE time; // 时间

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保内存对齐
    } EIGEN_ALIGN16;  // 强制 16 字节对齐

}  // namespace lslidar_driver

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_driver::PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (POINT_TIME_TYPE, time, time)
)

#endif  // LSLIDAR_POINTCLOUD_HPP
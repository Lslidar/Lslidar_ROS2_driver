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

#ifndef POINTCLOUD_TRANSFORM_HPP
#define POINTCLOUD_TRANSFORM_HPP

#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudTransform {
public:
    PointCloudTransform(double x_offset = 0.0, double y_offset = 0.0, double z_offset = 0.0,
                        double roll = 0.0, double pitch = 0.0, double yaw = 0.0);

    void setTransform(double x_offset, double y_offset, double z_offset,
                      double roll, double pitch, double yaw);

    template <typename PointCloudType>
    void applyTransform(PointCloudType& point_cloud) const;

private:
    void initializeTransformMatrix();

    double x_offset_;
    double y_offset_;
    double z_offset_;
    double roll_;
    double pitch_;
    double yaw_;

    Eigen::Matrix4f transform_matrix_;
};

inline PointCloudTransform::PointCloudTransform(double x_offset, double y_offset, double z_offset,
                                                 double roll, double pitch, double yaw)
    : x_offset_(x_offset), y_offset_(y_offset), z_offset_(z_offset),
      roll_(roll), pitch_(pitch), yaw_(yaw) {
    initializeTransformMatrix();
}

inline void PointCloudTransform::setTransform(double x_offset, double y_offset, double z_offset,
                                               double roll, double pitch, double yaw) {
    x_offset_ = x_offset;
    y_offset_ = y_offset;
    z_offset_ = z_offset;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
    initializeTransformMatrix();
}

inline void PointCloudTransform::initializeTransformMatrix() {
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix =
        Eigen::AngleAxisf(roll_, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(pitch_, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(yaw_, Eigen::Vector3f::UnitZ());

    transform_matrix_.setIdentity();
    transform_matrix_.block<3, 3>(0, 0) = rotation_matrix;
    transform_matrix_(0, 3) = x_offset_;
    transform_matrix_(1, 3) = y_offset_;
    transform_matrix_(2, 3) = z_offset_;
}

template <typename PointCloudType>
inline void PointCloudTransform::applyTransform(PointCloudType& point_cloud) const {
    pcl::transformPointCloud(point_cloud, point_cloud, transform_matrix_);
}

#endif  // POINTCLOUD_TRANSFORM_HPP

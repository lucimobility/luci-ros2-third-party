// Copyright (c) 2026 LUCI Mobility, Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>

#include <luci_messages/msg/luci_camera_info.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

/// @brief Realsense camera intrinsic values.
struct CameraIntrinsics
{
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

/// @brief LUCI camera locations from the chair center.
struct CameraTransform
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

/// Converts the three depth cameras on a LUCI wheelchair into a single combined point cloud in
/// `base_link`.
class FullCameraPointcloudNode : public rclcpp::Node
{
public:
    explicit FullCameraPointcloudNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    /// @brief Fires once per matched, time-synchronized triple of depth frames.
    /// @param leftDepthImage
    /// @param rightDepthImage
    /// @param rearDepthImage
    void onDepthFramesSynced(
        const sensor_msgs::msg::Image::ConstSharedPtr & leftDepthImage,
        const sensor_msgs::msg::Image::ConstSharedPtr & rightDepthImage,
        const sensor_msgs::msg::Image::ConstSharedPtr & rearDepthImage);

    /// @brief Get camera intrinsics & transform data.
    /// @param msg
    /// @param intrinsics
    /// @param transform
    /// @param received
    /// @param cameraLocation
    void storeCameraInfo(
        const luci_messages::msg::LuciCameraInfo::SharedPtr & msg,
        CameraIntrinsics & intrinsics,
        CameraTransform & transform,
        bool & received,
        const std::string & cameraLocation);

    /// @brief Subscribes to each camera's LuciCameraInfo topic.
    /// @param topic
    /// @param intrinsics
    /// @param transform
    /// @param received
    /// @param cameraLocation
    /// @return
    rclcpp::Subscription<luci_messages::msg::LuciCameraInfo>::SharedPtr subscribeCameraInfo(
        const std::string & topic,
        CameraIntrinsics & intrinsics,
        CameraTransform & transform,
        bool & received,
        const std::string & cameraLocation);

    /// @brief Converts a depth image straight into a base_link-frame point cloud.
    /// @param depthImage
    /// @param intrinsics
    /// @param tf
    /// @param cameraLocation
    /// @return
    sensor_msgs::msg::PointCloud2 convertDepthToBaseLinkPointCloud(
        const sensor_msgs::msg::Image::ConstSharedPtr & depthImage,
        const CameraIntrinsics & intrinsics,
        const CameraTransform & tf,
        const std::string & cameraLocation);

    /// @brief Concatenates all three base_link-frame camera clouds into one, stamped
    /// with the most recent of the three capture times.
    /// @param leftCloud
    /// @param rightCloud
    /// @param rearCloud
    /// @param timestamp
    /// @return
    sensor_msgs::msg::PointCloud2 combinePointClouds(
        const sensor_msgs::msg::PointCloud2 & leftCloud,
        const sensor_msgs::msg::PointCloud2 & rightCloud,
        const sensor_msgs::msg::PointCloud2 & rearCloud,
        const rclcpp::Time & timestamp);

    // Depth image subscribers.
    message_filters::Subscriber<sensor_msgs::msg::Image> depthLeftSub;
    message_filters::Subscriber<sensor_msgs::msg::Image> depthRightSub;
    message_filters::Subscriber<sensor_msgs::msg::Image> depthRearSub;

    // LuciCameraInfo subscribers.
    rclcpp::Subscription<luci_messages::msg::LuciCameraInfo>::SharedPtr leftInfoSub;
    rclcpp::Subscription<luci_messages::msg::LuciCameraInfo>::SharedPtr rightInfoSub;
    rclcpp::Subscription<luci_messages::msg::LuciCameraInfo>::SharedPtr rearInfoSub;

    // Camera intrinsics.
    CameraIntrinsics leftIntrinsics;
    CameraIntrinsics rightIntrinsics;
    CameraIntrinsics rearIntrinsics;

    // Camera transform values.
    CameraTransform leftTransform;
    CameraTransform rightTransform;
    CameraTransform rearTransform;

    // LuciCameraInfo received status for all cameras.
    bool leftCameraInfoReceived = false;
    bool rightCameraInfoReceived = false;
    bool rearCameraInfoReceived = false;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

    // Combined pointcloud publisher.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcCombinedPub;
};

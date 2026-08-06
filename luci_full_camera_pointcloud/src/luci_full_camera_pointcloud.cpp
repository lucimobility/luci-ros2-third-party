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

#include "luci_full_camera_pointcloud/luci_full_camera_pointcloud.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <string>

#include <sensor_msgs/image_encodings.hpp>

namespace
{
// Meters per raw depth count.
constexpr double DEPTH_SCALE = 0.0001;
constexpr int SYNC_QUEUE_SIZE = 5;
constexpr int PUBLISHER_QUEUE_SIZE = 3;
constexpr int CAMERA_INFO_QUEUE_SIZE = 1;
// Max allowed timestamp spread between the three camera frames combined into one point cloud.
constexpr double SYNC_SLOP_SECONDS = 0.05;
constexpr const char * COMBINED_FRAME_ID = "base_link";

/// @brief Checks that a depth Image message matches the 16-bit encoding.
bool isDepthImageValid(const sensor_msgs::msg::Image::ConstSharedPtr & depthImage)
{
    const bool encodingOk =
        depthImage->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        depthImage->encoding == sensor_msgs::image_encodings::MONO16;
    const bool stepOk = depthImage->step >= depthImage->width * sizeof(uint16_t);
    const bool sizeOk =
        stepOk &&
        depthImage->data.size() >= static_cast<size_t>(depthImage->step) * depthImage->height;

    return encodingOk && sizeOk;
}

/// @brief Sizes `cloud` as an organized (width x height) XYZ-float32 point cloud.
void setupOrganizedXyzCloud(sensor_msgs::msg::PointCloud2 & cloud, uint32_t width, uint32_t height)
{
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(static_cast<size_t>(width) * height);

    cloud.width = width;
    cloud.height = height;
    cloud.row_step = cloud.point_step * cloud.width;
}

/// @brief A camera-to-base_link rotation matrix + translation, as flattened scalars for the
/// per-pixel hot loop in convertDepthToBaseLinkPointCloud().
struct RotationMatrix
{
    float r00 = 0.0f;
    float r01 = 0.0f;
    float r02 = 0.0f;
    float r10 = 0.0f;
    float r11 = 0.0f;
    float r12 = 0.0f;
    float r20 = 0.0f;
    float r21 = 0.0f;
    float r22 = 0.0f;
    float tx = 0.0f;
    float ty = 0.0f;
    float tz = 0.0f;
};

/// @brief Composes a camera-to-base_link rotation matrix + translation from roll/pitch/yaw.
RotationMatrix computeRotationMatrix(const CameraTransform & tf)
{
    const float rollCos = std::cos(static_cast<float>(tf.roll));
    const float rollSin = std::sin(static_cast<float>(tf.roll));
    const float pitchCos = std::cos(static_cast<float>(tf.pitch));
    const float pitchSin = std::sin(static_cast<float>(tf.pitch));
    const float yawCos = std::cos(static_cast<float>(tf.yaw));
    const float yawSin = std::sin(static_cast<float>(tf.yaw));

    RotationMatrix rot;
    rot.r00 = yawCos * pitchCos;
    rot.r01 = yawCos * pitchSin * rollSin - yawSin * rollCos;
    rot.r02 = yawCos * pitchSin * rollCos + yawSin * rollSin;
    rot.r10 = yawSin * pitchCos;
    rot.r11 = yawSin * pitchSin * rollSin + yawCos * rollCos;
    rot.r12 = yawSin * pitchSin * rollCos - yawCos * rollSin;
    rot.r20 = -pitchSin;
    rot.r21 = pitchCos * rollSin;
    rot.r22 = pitchCos * rollCos;
    rot.tx = static_cast<float>(tf.x);
    rot.ty = static_cast<float>(tf.y);
    rot.tz = static_cast<float>(tf.z);
    return rot;
}
}  // namespace

FullCameraPointcloudNode::FullCameraPointcloudNode(const rclcpp::NodeOptions & options)
: Node("luci_full_camera_pointcloud", options)
{
    this->depthLeftSub.subscribe(this, "luci/depth_left_camera");
    this->depthRightSub.subscribe(this, "luci/depth_right_camera");
    this->depthRearSub.subscribe(this, "luci/depth_rear_camera");

    this->sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(SYNC_QUEUE_SIZE), this->depthLeftSub, this->depthRightSub, this->depthRearSub);
    this->sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(SYNC_SLOP_SECONDS));
    this->sync->registerCallback(
        std::bind(
            &FullCameraPointcloudNode::onDepthFramesSynced, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    this->leftInfoSub = this->subscribeCameraInfo(
        "luci/left_camera_info", this->leftIntrinsics, this->leftTransform,
        this->leftCameraInfoReceived, "left");
    this->rightInfoSub = this->subscribeCameraInfo(
        "luci/right_camera_info", this->rightIntrinsics, this->rightTransform,
        this->rightCameraInfoReceived, "right");
    this->rearInfoSub = this->subscribeCameraInfo(
        "luci/rear_camera_info", this->rearIntrinsics, this->rearTransform,
        this->rearCameraInfoReceived, "rear");

    this->pcCombinedPub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "luci/depth_combined_pointcloud", PUBLISHER_QUEUE_SIZE);

    RCLCPP_INFO(this->get_logger(), "luci_full_camera_pointcloud started");
}

rclcpp::Subscription<luci_messages::msg::LuciCameraInfo>::SharedPtr
FullCameraPointcloudNode::subscribeCameraInfo(
    const std::string & topic,
    CameraIntrinsics & intrinsics,
    CameraTransform & transform,
    bool & received,
    const std::string & cameraLocation)
{
    return this->create_subscription<luci_messages::msg::LuciCameraInfo>(
        topic, CAMERA_INFO_QUEUE_SIZE,
        [this, &intrinsics, &transform, &received, cameraLocation](
            const luci_messages::msg::LuciCameraInfo::SharedPtr msg) {
            this->storeCameraInfo(msg, intrinsics, transform, received, cameraLocation);
        });
}

void FullCameraPointcloudNode::storeCameraInfo(
    const luci_messages::msg::LuciCameraInfo::SharedPtr & msg,
    CameraIntrinsics & intrinsics,
    CameraTransform & transform,
    bool & received,
    const std::string & cameraLocation)
{
    // Once camera info has been received for this camera, ignore all further LuciCameraInfo
    // messages for it.
    if (received)
    {
        return;
    }

    if (msg->intrinsics.size() < 4 || msg->translation.size() < 3 || msg->rotation.size() < 3)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Ignoring LuciCameraInfo for '%s' camera: expected intrinsics[4]/translation[3]/"
            "rotation[3], got sizes [%zu]/[%zu]/[%zu]",
            cameraLocation.c_str(), msg->intrinsics.size(), msg->translation.size(),
            msg->rotation.size());
        return;
    }

    intrinsics.fx = msg->intrinsics[0];
    intrinsics.fy = msg->intrinsics[1];
    intrinsics.cx = msg->intrinsics[2];
    intrinsics.cy = msg->intrinsics[3];

    transform.x = msg->translation[0];
    transform.y = msg->translation[1];
    transform.z = msg->translation[2];

    transform.roll = msg->rotation[0];
    transform.pitch = msg->rotation[1];
    transform.yaw = msg->rotation[2];

    RCLCPP_INFO(
        this->get_logger(), "Received info for '%s' camera", cameraLocation.c_str());
    received = true;
}

void FullCameraPointcloudNode::onDepthFramesSynced(
    const sensor_msgs::msg::Image::ConstSharedPtr & leftDepthImage,
    const sensor_msgs::msg::Image::ConstSharedPtr & rightDepthImage,
    const sensor_msgs::msg::Image::ConstSharedPtr & rearDepthImage)
{
    // Do not process or publish anything until we have all three cameras info.
    if (!this->leftCameraInfoReceived || !this->rightCameraInfoReceived ||
        !this->rearCameraInfoReceived)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Not publishing: waiting for camera info  (left=%s, right=%s, rear=%s)",
            this->leftCameraInfoReceived ? "ready" : "waiting",
            this->rightCameraInfoReceived ? "ready" : "waiting",
            this->rearCameraInfoReceived ? "ready" : "waiting");
        return;
    }

    const auto leftPointcloud = this->convertDepthToBaseLinkPointCloud(
        leftDepthImage, this->leftIntrinsics, this->leftTransform, "left");
    const auto rightPointcloud = this->convertDepthToBaseLinkPointCloud(
        rightDepthImage, this->rightIntrinsics, this->rightTransform, "right");
    const auto rearPointcloud = this->convertDepthToBaseLinkPointCloud(
        rearDepthImage, this->rearIntrinsics, this->rearTransform, "rear");

    // Use the most recent timestamp.
    rclcpp::Time timestamp = leftDepthImage->header.stamp;
    if (rclcpp::Time(rightDepthImage->header.stamp) > timestamp)
    {
        timestamp = rightDepthImage->header.stamp;
    }
    if (rclcpp::Time(rearDepthImage->header.stamp) > timestamp)
    {
        timestamp = rearDepthImage->header.stamp;
    }

    const auto combined = this->combinePointClouds(
        leftPointcloud, rightPointcloud, rearPointcloud, timestamp);

    this->pcCombinedPub->publish(combined);
}

sensor_msgs::msg::PointCloud2 FullCameraPointcloudNode::convertDepthToBaseLinkPointCloud(
    const sensor_msgs::msg::Image::ConstSharedPtr & depthImage,
    const CameraIntrinsics & intrinsics,
    const CameraTransform & tf,
    const std::string & cameraLocation)
{
    sensor_msgs::msg::PointCloud2 pointcloud;
    pointcloud.header.frame_id = COMBINED_FRAME_ID;
    pointcloud.is_dense = false;
    pointcloud.is_bigendian = false;

    if (!isDepthImageValid(depthImage))
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Rejecting depth frame on '%s': unsupported encoding or malformed image data - "
            "producing an empty cloud for this frame",
            cameraLocation.c_str());
        setupOrganizedXyzCloud(pointcloud, 0, 0);
        return pointcloud;
    }

    setupOrganizedXyzCloud(pointcloud, depthImage->width, depthImage->height);

    sensor_msgs::PointCloud2Iterator<float> iterX(pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pointcloud, "z");

    const uint8_t * data = depthImage->data.data();

    const float depthScale = static_cast<float>(DEPTH_SCALE);
    const float cx = static_cast<float>(intrinsics.cx);
    const float cy = static_cast<float>(intrinsics.cy);
    const float fxInv = 1.0f / static_cast<float>(intrinsics.fx);
    const float fyInv = 1.0f / static_cast<float>(intrinsics.fy);

    const RotationMatrix rot = computeRotationMatrix(tf);

    for (uint32_t v = 0; v < depthImage->height; ++v)
    {
        const uint16_t * depthRow =
            reinterpret_cast<const uint16_t *>(data + static_cast<size_t>(v) * depthImage->step);

        for (uint32_t u = 0; u < depthImage->width; ++u, ++iterX, ++iterY, ++iterZ)
        {
            uint16_t raw = depthRow[u];
            if (raw == 0)
            {
                *iterX = *iterY = *iterZ = std::numeric_limits<float>::quiet_NaN();
                continue;
            }

            const float depthZ = static_cast<float>(raw) * depthScale;
            const float camX = (static_cast<float>(u) - cx) * depthZ * fxInv;
            const float camY = (static_cast<float>(v) - cy) * depthZ * fyInv;

            *iterX = rot.r00 * camX + rot.r01 * camY + rot.r02 * depthZ + rot.tx;
            *iterY = rot.r10 * camX + rot.r11 * camY + rot.r12 * depthZ + rot.ty;
            *iterZ = rot.r20 * camX + rot.r21 * camY + rot.r22 * depthZ + rot.tz;
        }
    }

    return pointcloud;
}

sensor_msgs::msg::PointCloud2 FullCameraPointcloudNode::combinePointClouds(
    const sensor_msgs::msg::PointCloud2 & leftCloud,
    const sensor_msgs::msg::PointCloud2 & rightCloud,
    const sensor_msgs::msg::PointCloud2 & rearCloud,
    const rclcpp::Time & timestamp)
{
    sensor_msgs::msg::PointCloud2 combined;
    combined.header.stamp = timestamp;
    combined.header.frame_id = COMBINED_FRAME_ID;
    combined.fields = leftCloud.fields;
    combined.point_step = leftCloud.point_step;
    combined.is_bigendian = leftCloud.is_bigendian;
    combined.is_dense = false;

    const size_t totalPoints =
        static_cast<size_t>(leftCloud.width * leftCloud.height) +
        static_cast<size_t>(rightCloud.width * rightCloud.height) +
        static_cast<size_t>(rearCloud.width * rearCloud.height);

    combined.data.reserve(totalPoints * leftCloud.point_step);
    combined.data.insert(combined.data.end(), leftCloud.data.begin(), leftCloud.data.end());
    combined.data.insert(combined.data.end(), rightCloud.data.begin(), rightCloud.data.end());
    combined.data.insert(combined.data.end(), rearCloud.data.begin(), rearCloud.data.end());

    combined.width = static_cast<uint32_t>(totalPoints);
    combined.height = 1;
    combined.row_step = combined.point_step * combined.width;

    return combined;
}

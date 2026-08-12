# luci_full_camera_pointcloud

Converts the three depth image streams from a LUCI-equipped wheelchair's left, right, and rear
cameras into a single combined `sensor_msgs/PointCloud2` in the chair's `base_link` frame.

## Architecture

```text
luci/depth_left_camera   ─┐
luci/depth_right_camera  ─┼─► ApproximateTime sync ─► matched (left, right, rear)
luci/depth_rear_camera   ─┘                                       │
                                                                   ▼
                                       per camera: convertDepthToBaseLinkPointCloud()
                                       (back-project depth image -> XYZ points, then
                                        rotate + translate into base_link, in one pass)
                                                                   │
                                                                   ▼
                                                    combinePointClouds()
                                              (concatenate all three into one cloud)
                                                                   │
                                                                   ▼
                                         luci/depth_combined_pointcloud (base_link, all three)
```

## Topics

### Subscribed

| Topic | Type | Description |
| --- | --- | --- |
| `luci/depth_left_camera` | `sensor_msgs/Image` (16UC1) | Left camera raw depth |
| `luci/depth_right_camera` | `sensor_msgs/Image` (16UC1) | Right camera raw depth |
| `luci/depth_rear_camera` | `sensor_msgs/Image` (16UC1) | Rear camera raw depth |
| `luci/left_camera_info` | `luci_messages/LuciCameraInfo` | Left-camera information |
| `luci/right_camera_info` | `luci_messages/LuciCameraInfo` | Right-camera information |
| `luci/rear_camera_info` | `luci_messages/LuciCameraInfo` | Rear-camera information |

### Published

| Topic | Frame | Description |
| --- | --- | --- |
| `luci/depth_combined_pointcloud` | `base_link` (configurable) | All three cameras, transformed and concatenated |

## Build

```bash
colcon build --packages-select luci_full_camera_pointcloud
```

## Run

```bash
ros2 launch luci_full_camera_pointcloud luci_full_camera_pointcloud.launch.py
```

## Tuning

These are compile-time constants at the top of `src/luci_full_camera_pointcloud.cpp`

| Constant | Value | Meaning |
| --- | --- | --- |
| `DEPTH_SCALE` | `0.0001` | Meters per raw depth count (i.e. each unit in the 16-bit depth image is 0.1mm). |
| `SYNC_SLOP_SECONDS` | `0.05` | Max allowed `header.stamp` spread between the three depth frames for `ApproximateTime` to consider them a match. |
| `SYNC_QUEUE_SIZE` | `5` | Depth-frame history kept per camera by the `message_filters::Synchronizer` while it searches for a match. |
| `CAMERA_INFO_QUEUE_SIZE` | `1` | Queue depth for the `LuciCameraInfo` subscriptions; only the first valid message per camera is ever used (see `storeCameraInfo`'s latch). |
| `PUBLISHER_QUEUE_SIZE` | `3` | Queue depth for the `luci/depth_combined_pointcloud` publisher. |

## Known limitations

- No overlap handling when concatenating the three cameras into `luci/depth_combined_pointcloud`:
  clouds are appended as-is, so physically overlapping fields of view between adjacent cameras
  will show up as duplicate/overlapping.

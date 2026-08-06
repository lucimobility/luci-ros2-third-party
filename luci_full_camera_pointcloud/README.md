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

## Known limitations

- No overlap handling when concatenating the three cameras into `luci/depth_combined_pointcloud`:
  clouds are appended as-is, so physically overlapping fields of view between adjacent cameras
  will show up as duplicate/overlapping.

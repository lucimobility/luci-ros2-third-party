# NOTICE

This repository includes third-party software and assets. Below are the
third-party projects and assets that are included or referenced by this
repository, with notes about authorship and licensing where available.

## ROS 2 (core packages and client libraries)

- **Project**: ROS 2 (Robot Operating System 2)
- **Description**: Middleware, client libraries, and tools used by the
  packages and launch files in this repository. This includes, but is not
  limited to, `rclcpp`, `geometry_msgs`, `std_msgs`, `xacro`, `rviz` and the
  `ament` build tooling referenced in `package.xml` and `CMakeLists.txt`.
- **License**: Apache License 2.0
- **Notes**: Full text of the Apache License 2.0 is included in
  `LICENSE.md` at the repository root. Individual ROS 2 packages may include
  additional license/NOTICE files — see the upstream package sources for
  details.

---

## message_filters

- **Project**: [message_filters](https://github.com/ros2/message_filters)
- **Description**: Used by `luci_full_camera_pointcloud` to time-synchronize the three depth camera
  streams (`message_filters::sync_policies::ApproximateTime`) before combining them into one
  point cloud.
- **License**: BSD-3-Clause (per upstream — distinct from the blanket ROS 2/Apache-2.0 entry
  above; verify against the copy vendored in your ROS 2 distribution).
- **Notes**: See [the upstream LICENSE file](https://github.com/ros2/message_filters/blob/rolling/LICENSE)
  for the full license text.

---

## COLLADA meshes (3D assets)

- **Files**: `meshes/main_wheel.dae`, `meshes/caster_wheels.dae`
- **Author / Asset Metadata**: The COLLADA files include contributor
  metadata identifying the author as "SimLab Soft" and authoring tool as
  "SimLab Composer" (see asset headers inside the `.dae` files).
- **License**: Not specified in-repo. The asset files contain author
  metadata but do not include an explicit license text. If you plan to
  redistribute these mesh files (for example, as part of a binary bundle),
  verify the licensing terms with the asset author or source and add the
  appropriate attribution and license text to this repository.

---

## Repository license

- **This repository**: Apache License 2.0 (see `LICENSE.md`).

If you believe a third-party component or asset is missing from this list,
or you have additional license information for any entry above, please open a PR to
update this file so license/attribution information is complete and
accurate.
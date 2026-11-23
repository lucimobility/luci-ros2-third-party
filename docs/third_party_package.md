# luci_third_party

This page covers the root `luci_third_party` package and its two launch files. The other packages
in this repo have their own READMEs, linked from the [root README](../README.md).

After correctly sourcing ROS2, use the launch files below.

## Wheelchair launch file

```bash
ros2 launch luci_third_party wheelchair.launch.py ip:=<chair ip address>
```

Launches:

- The gRPC interface node (from `luci_grpc_interface`). The required `ip` launch argument is the
  address of the chair to connect to, and is passed to the node with its `-a` flag.
- The LUCI transform node (from `luci_transforms`), which publishes the transforms between
  `base_link` and all the sensors. The transforms are for the Quickie 500M wheelchair.

## URDF launch file

```bash
ros2 launch luci_third_party awl_description.launch.py
```

Launches:

- The joint state publisher, which publishes the joints from the xacro file.
- The robot state publisher, which publishes the description of the robot.
- rviz2, using the rviz config to show the robot description topic. The `rviz_config_file` launch
  argument sets the config file, and defaults to the installed `config/luci_view.rviz`.

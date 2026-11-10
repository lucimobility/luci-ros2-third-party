# luci-third-party

## Wheelchair launch file

After correctly sourcing ROS2, run the following to start the node:

```bash
ros2 launch luci_third_party wheelchair.launch.py ip:={Chair IP address}
```

This launch files starts up the gRPC interface node and transformation node. The ip launch argument is used to specify the specific chair to connect to.

## URDF launch file

Run the following command:

```bash
ros2 launch luci_third_party awl_description.launch.py
```

The launch file starts the joint state publisher and robot state publisher for the urdf as well as showing it in rviz.

## Twist to joystick converter

First start the chair interface with the wheelchair launch file above, since this node only publishes ROS topics and relies on the gRPC interface node to reach the chair. Then, after building and sourcing the workspace, run:

```bash
ros2 launch twist_to_joystick_converter twist_to_joystick.launch.py
```

This converts `cmd_vel` (`geometry_msgs/Twist`) into `luci/remote_joystick` messages with the remote input source set. It does not activate or remove remote control on the chair: you do that by hand with the interface's services, and LUCI ignores the commands until a remote input source is active. The package README has the commands. The calibration is for the Q500M and is not yet validated on a chair, so read the [package README](../twist_to_joystick_converter/README.md) before driving.

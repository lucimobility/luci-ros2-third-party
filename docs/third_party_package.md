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

The `twist_to_joystick_converter` package converts `cmd_vel` (`geometry_msgs/Twist`) into `luci/remote_joystick` messages. Start the chair interface with the wheelchair launch file above first, then run:

```bash
ros2 launch twist_to_joystick_converter twist_to_joystick.launch.py
```

The calibration was validated on a Quickie Q500M (drive profile 3, speed setting 3, one rider) and only holds for that chair, profile and speed setting. See the [package README](../twist_to_joystick_converter/README.md) for setup, remote control activation and safety notes before driving.

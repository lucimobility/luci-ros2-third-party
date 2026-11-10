
# Launch Files

## awl_wheelchair launch file

```bash
ros2 launch luci_third_party wheelchair.launch.py
```

Launches

- grpc interface node (from luci_grpc_interface)
  - uses flag -a to set the chair ip address, which is 192.168.8.203
- luci tf node between base_link and all the sensors (from luci_transforms)
  - transforms are for the quickie 500m wheelchair but can also be for permobil m3

## awl_description launch file

```bash
ros2 launch luci_third_party awl_description.launch.py
```

Launches

- joint state publisher (publishes the joints from the xacro file)
- robot state publisher (publishes the description of the robot)
- rviz2 using the rviz config to see the robot description topic

## twist_to_joystick launch file

```bash
ros2 launch twist_to_joystick_converter twist_to_joystick.launch.py
```

This launch file lives in the `twist_to_joystick_converter` package, not in `luci_third_party`. It is not started by `wheelchair.launch.py`, which must be running as well so that the grpc interface node can pass the joystick commands on to the chair. The node does not activate remote control, so LUCI ignores its commands until you activate a remote input source by hand. The [package README](../twist_to_joystick_converter/README.md) has the commands.

Launches

- twist to joystick node (converts `cmd_vel` into `luci/remote_joystick`)
  - `params_file` sets the calibration, default is the Q500M `config/q500m.yaml`
  - `cmd_vel_topic` sets the Twist topic to listen to

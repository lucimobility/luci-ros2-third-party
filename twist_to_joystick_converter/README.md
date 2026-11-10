# twist_to_joystick_converter

Converts `geometry_msgs/Twist` velocity commands (`cmd_vel`) into `luci_messages/LuciJoystick`
commands so that standard ROS 2 navigation stacks and teleop tools can drive a LUCI-equipped
wheelchair. It is intended as a starting point for researchers.

> **Safety.** This node produces joystick commands for a real wheelchair.
>
> - The calibration constants are for the Quickie Q500M and are **not yet validated on a chair**.
> - The node does **not** activate remote control, and it does **not** hand control back when it
>   exits. You activate and remove it by hand (see
>   [Activating remote control](#activating-remote-control)). LUCI ignores the commands until a
>   remote input source is active.
> - While autonomous remote control is active, the chair's physical joystick does not work.
>   Remove it when you are done.
> - It does not send a zero command when it exits. What the chair does once commands stop is up
>   to LUCI and the chair, so let the chair come to a stop (zeros are published within
>   `cmd_vel_timeout` of the last `cmd_vel`) before you stop the node.
>
> Test at low speed with an e-stop within reach.

## Prerequisites

- A ROS 2 workspace (or install) that provides `luci_messages`: the `ros-humble-luci-messages`
  apt package, or the [luci-ros2-msgs](https://github.com/lucimobility/luci-ros2-msgs) repository.
- The LUCI gRPC interface node must be running. This package only publishes ROS topics; it is
  the interface node that passes `luci/remote_joystick` on to the chair. Start it from the repo
  root launch file:

  ```bash
  ros2 launch luci_third_party wheelchair.launch.py ip:=<chair ip address>
  ```

## Activating remote control

This node does not activate remote control or give it back. You do that with the gRPC interface's
services, with the interface node running. Use the pair that matches the `input_source` parameter,
or LUCI disregards the commands:

| `input_source` | Activate | Remove |
| --- | --- | --- |
| `1` (autonomous remote, default) | `ros2 service call /luci/set_auto_remote_input std_srvs/srv/Empty` | `ros2 service call /luci/remove_auto_remote_input std_srvs/srv/Empty` |
| `5` (shared remote) | `ros2 service call /luci/set_shared_remote_input std_srvs/srv/Empty` | `ros2 service call /luci/remove_shared_remote_input std_srvs/srv/Empty` |

- In autonomous remote (`1`) the chair's physical joystick is disabled until you remove the
  source. In shared remote (`5`) it still streams.
- Remove the source when you are finished, so the physical joystick works again. The node does not
  do this when it exits.
- LUCI needs joystick commands faster than 10 Hz. This node publishes at 20 Hz.
- The SDK's keyboard teleop,
  [luci-ros2-keyboard-teleop](https://github.com/lucimobility/luci-ros2-keyboard-teleop), makes the
  same calls automatically, if you want a reference for doing that inside a node.

## Architecture

```text
cmd_vel (Twist) ─► twist_to_joystick() ─► latest command
                    cubic fit (linear.x -> forward_back)
                    linear fit (angular.z -> left_right)
                    clamp to [js_min, js_max]

20 Hz timer ─► latest command (or zero if no cmd_vel within cmd_vel_timeout)
            ─► luci/remote_joystick (LuciJoystick, input_source set)
```

The conversion itself lives in `twist_to_joystick_converter/conversion.py` and has no ROS
dependency.

## Topics

### Subscribed

| Topic | Type | Description |
| --- | --- | --- |
| `cmd_vel` | `geometry_msgs/Twist` | Velocity command. Only `linear.x` and `angular.z` are used |

### Published

| Topic | Type | Description |
| --- | --- | --- |
| `luci/remote_joystick` | `luci_messages/LuciJoystick` | Joystick command in `[js_min, js_max]` with `input_source` set |

### The LuciJoystick message

| Field | What this node sends |
| --- | --- |
| `forward_back` | `[js_min, js_max]`, default `[-100, 100]`. The gRPC interface adds 100 before sending, so this is already the right range |
| `left_right` | Same range and handling as `forward_back` |
| `input_source` | The `input_source` parameter: `1` = autonomous remote (default), `5` = shared remote. LUCI only uses commands from an input source that has been activated |
| `joystick_zone` | Left at its default. The interface does not send it, and LUCI works the zone out from the two axes |

## Behavior

- **Conversion.** Forward/back is `c3 * v^3 - c2 * v^2 + c1 * v` with `v = linear.x` in m/s.
  Left/right is `(w - angular_offset) / angular_slope` with `w = angular.z` in rad/s. Both are
  truncated toward zero and clamped to `[js_min, js_max]`. A zero input gives zero output.
- **Bad input.** NaN, infinity, or a magnitude above 100 m/s or rad/s gives a zero joystick
  command and a warning, never a crash.
- **Publishing.** The node publishes the latest command at 20 Hz, whether or not new `cmd_vel`
  messages arrive. If no `cmd_vel` has arrived within `cmd_vel_timeout`, it publishes zero instead
  until the next one. This also holds from node start, so the node publishes zeros on
  `luci/remote_joystick` even before the first `cmd_vel`. Do not run another joystick publisher on
  that topic at the same time, because its commands would be interleaved with these zeros.
- **Keep publishing.** Because a stale command becomes zero, send `cmd_vel` continuously (10 Hz
  or more is typical). A single message only moves the chair for `cmd_vel_timeout` seconds.

### About the calibration

The constants are an open-loop fit from joystick percent to chair speed. Read these before
trusting the numbers:

- **Sign.** A positive `angular.z` (counter-clockwise) gives a negative `left_right`.
- **Range.** Forward reaches about 1.05 m/s at `forward_back` 100, but reverse only about
  0.32 m/s at -100. Commands beyond that saturate at the limit.
- **Speed setting.** Joystick percent is a fraction of the chair's maximum speed, which depends
  on the chair's speed setting and drive profile (`luci/speed_setting`, `luci/chair_profile`).
  The values only hold for the setting they were measured at. LUCI's obstacle scaling can also
  lower the speed the chair actually drives.
- **Cubic vs. linear.** The forward/back cubic was fitted to two earlier linear fits:
  `(v + 0.0206) / 0.010724` for forward and `(v - 0.0004) / 0.00316` for reverse. The cubic
  matches them at -0.3, 0.5 and 1.0 m/s, but differs by up to about 10 joystick units in between
  (for example 0.2 m/s gives 29.7 from the cubic and 20.6 from the linear fit). Which one is
  closer to the chair is not yet known.
- **Small yaw.** Because of `angular_offset`, any nonzero yaw gives about +5 on `left_right`, in
  the same direction whichever way the yaw points (`1e-6` and `-1e-6` rad/s both give 5). Only an
  exact `0.0` gives 0. A planner that publishes a tiny nonzero yaw when driving "straight" will
  make the chair turn slightly.

## Parameters

Defaults are the Q500M values in `config/q500m.yaml`. All parameters are **read once at
startup**: `ros2 param set` on them is rejected, so change them in a params file and restart. Values
may be written as integers or floats in YAML. The node exits with an error if a value is invalid:
not a number or not finite, an `angular_slope` of zero, constants that overflow, `js_min` or
`js_max` that is not a whole number inside `[-100, 100]` (or `js_min > js_max`), a
`cmd_vel_timeout` above 3600, or an `input_source` other than 1 or 5.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `forward_coeff_3` | `208.6` | Cubic term of the forward/back fit |
| `forward_coeff_2` | `316.7` | Quadratic term (subtracted) |
| `forward_coeff_1` | `203.3` | Linear term |
| `angular_offset` | `0.0676` | Offset in the left/right fit |
| `angular_slope` | `-0.012212` | Slope in the left/right fit |
| `js_min` | `-100` | Lower joystick limit (whole number, `>= -100`) |
| `js_max` | `100` | Upper joystick limit (whole number, `<= 100`) |
| `cmd_vel_timeout` | `0.5` | Seconds without `cmd_vel` before zero joystick is published. `<= 0` disables this; at most `3600` |
| `input_source` | `1` | `LuciJoystick.input_source`: `1` = autonomous remote, `5` = shared remote |

### Using your own calibration

Copy `config/q500m.yaml`, edit the values, and pass it to the launch file:

```bash
ros2 launch twist_to_joystick_converter twist_to_joystick.launch.py params_file:=/path/to/my_chair.yaml
```

## Build

Run these from the repository root. A plain `colcon build` there only builds the root
`luci_third_party` package, because colcon does not search for packages inside a directory that is
already a package. Point colcon at this package explicitly with `--paths`:

```bash
colcon build --paths twist_to_joystick_converter
source install/setup.bash
```

This package is built from source with colcon. It is not part of the `.deb` that the repo's
`build-package.sh` produces.

## Run

```bash
ros2 launch twist_to_joystick_converter twist_to_joystick.launch.py
```

Launch arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `params_file` | installed `config/q500m.yaml` | YAML file with the parameters above |
| `cmd_vel_topic` | `cmd_vel` | Twist topic to subscribe to |

### Try it

In one terminal, watch the output:

```bash
ros2 topic echo luci/remote_joystick
```

In another, publish a slow forward command continuously:

```bash
ros2 topic pub -r 10 cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

Stop the publisher and the output returns to zero after `cmd_vel_timeout`.

The chair only responds once a remote input source has been activated (see
[Activating remote control](#activating-remote-control)).

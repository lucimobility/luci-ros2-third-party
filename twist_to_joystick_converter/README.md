# twist_to_joystick_converter

Converts `geometry_msgs/Twist` velocity commands (`cmd_vel`) into `luci_messages/LuciJoystick`
commands so that standard ROS 2 navigation stacks and teleop tools can drive a LUCI-equipped
wheelchair. It is intended as a starting point for researchers.

> **Safety.** This node produces joystick commands for a real wheelchair.
>
> - The calibration constants are for the Quickie Q500M **drive
>   profile 3 at speed setting 3**.
> - The calibration only holds for the chair, drive profile and speed setting it was measured at.
>   You are encouraged to measure your own
>   and keep a params file for each chair, profile and speed setting you use (see
>   [Using your own calibration](#using-your-own-calibration)). Contributions are welcome.
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
                    line through zero (linear.x -> forward_back)
                    dead zone, then a line (angular.z -> left_right)
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

- **Conversion.** Forward/back is `100 * v / max_forward_speed` for `v >= 0` and
  `100 * v / max_reverse_speed` for `v < 0`, with `v = linear.x` in m/s: the joystick value is the
  percentage of the profile's top speed. Left/right is, for a nonzero `w = angular.z` in rad/s,
  `turn_dead_zone + (100 - turn_dead_zone) * |w| / max_turn_rate` with the sign flipped
  (a positive `angular.z` gives a negative `left_right`). Both are rounded to the nearest whole
  number and clamped to `[js_min, js_max]`. A zero input gives zero output.
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

The constants are an open-loop fit from joystick percent to chair speed. They were measured on
profile 3 at speed setting 3. Read these before trusting the numbers:

- **Sign.** A positive `angular.z` (counter-clockwise) gives a negative `left_right`.
- **Speed setting and profile.** The chair's top speeds depend on the speed setting and drive
  profile (`luci/speed_setting`, `luci/chair_profile`). The values only hold for the setting they
  were measured at: profile 3 at speed setting 3. LUCI's obstacle scaling can also lower the speed
  the chair actually drives.
- **Load.** The readings were taken with one rider. A different load may change the chair's
  response, especially for turning.
- **Small yaw.** Any nonzero yaw gives at least `turn_dead_zone` on `left_right`, in the
  matching direction (`1e-6` rad/s gives -18 and `-1e-6` gives 18). Only an exact `0.0` gives 0.
  The chair starts to turn at about joystick 18 (`turn_dead_zone`), but it only holds a turn
  from about joystick 25. A planner that publishes a tiny nonzero yaw when driving "straight"
  therefore will normally not make it turn.

## Parameters

Defaults are the Q500M values in `config/q500m.yaml`. All parameters are **read once at
startup**: `ros2 param set` on them is rejected, so change them in a params file and restart. Values
may be written as integers or floats in YAML. The node exits with an error if a value is invalid:
not a number or not finite, a speed or turn rate that is not above zero, a `turn_dead_zone` outside
`[0, 100)`, constants that overflow, `js_min` or
`js_max` that is not a whole number inside `[-100, 100]` (or `js_min > js_max`), a
`cmd_vel_timeout` that is not above 0 and at most 2, or an `input_source` other than 1 or 5.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `max_forward_speed` | `1.028` | The chair's top forward speed in m/s (2.3 mph), which is joystick 100 |
| `max_reverse_speed` | `0.313` | The chair's top reverse speed in m/s (0.7 mph), which is joystick -100 |
| `max_turn_rate` | `0.524` | Turn rate in rad/s at joystick 100 |
| `turn_dead_zone` | `18.0` | Joystick units below which the chair does not turn; any nonzero yaw gives at least this. The chair only holds a turn from about 25 (see Small yaw above) |
| `js_min` | `-100` | Lower joystick limit (whole number, `>= -100`) |
| `js_max` | `100` | Upper joystick limit (whole number, `<= 100`) |
| `cmd_vel_timeout` | `0.5` | Seconds without `cmd_vel` before zero joystick is published. Must be above `0` and at most `2` |
| `input_source` | `1` | `LuciJoystick.input_source`: `1` = autonomous remote, `5` = shared remote |

### Using your own calibration

Copy `config/q500m.yaml`, edit the values, and pass it to the launch file:

```bash
ros2 launch twist_to_joystick_converter twist_to_joystick.launch.py params_file:=/path/to/my_chair.yaml
```

To get the values for your chair, profile and speed setting:

1. **Top speeds.** Read the maximum forward and reverse speed from the drive profile (the chair's
   programmer or remote). Convert mph to m/s, and set `max_forward_speed` and `max_reverse_speed`. Check them by publishing a few joystick values straight to
   `luci/remote_joystick` and reading the chair display.
2. **Turning.** Send `left_right` values to the same topic and time a turn against a mark on the
   floor. Find the lowest value that keeps the chair turning (`turn_dead_zone`) and the rate at
   100 (`max_turn_rate`). Check a few values in between.
3. **Test with a rider if the chair will carry one**, because the response can change with load.

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

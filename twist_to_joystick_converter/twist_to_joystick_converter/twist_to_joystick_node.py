# Copyright (c) 2026 LUCI Mobility, Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Node that converts cmd_vel Twist messages to LuciJoystick."""

from dataclasses import fields
import math
import sys

from geometry_msgs.msg import Twist
from luci_messages.msg import LuciJoystick
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node

from twist_to_joystick_converter.conversion import (
    ConversionParams, is_valid_input, twist_to_joystick, ZERO)

# Publish period, 20 Hz (LUCI needs more than 10 Hz).
PUBLISH_PERIOD_S = 0.05

# Maximum cmd_vel_timeout, in seconds.
MAX_CMD_VEL_TIMEOUT_S = 3600.0

# LuciJoystick.input_source values for remote commands.
INPUT_SOURCE_AUTONOMOUS_REMOTE = 1
INPUT_SOURCE_SHARED_REMOTE = 5
VALID_INPUT_SOURCES = (INPUT_SOURCE_AUTONOMOUS_REMOTE,
                       INPUT_SOURCE_SHARED_REMOTE)


class ConfigurationError(Exception):
    """Raised when a node parameter is invalid."""


class JoystickConverter(Node):
    """Publish LuciJoystick commands converted from cmd_vel."""

    def __init__(self):
        super().__init__('twist_to_joystick_node')

        # One parameter per ConversionParams field, with the same default.
        try:
            self.params_ = ConversionParams(**{
                field.name: self._declare(field.name, field.default)
                for field in fields(ConversionParams)})
        except ValueError as error:
            raise ConfigurationError(str(error)) from error

        # Seconds without cmd_vel before publishing zero; <= 0 disables.
        self.cmd_vel_timeout_ = self._declare('cmd_vel_timeout', 0.5)
        if (not math.isfinite(self.cmd_vel_timeout_) or
                self.cmd_vel_timeout_ > MAX_CMD_VEL_TIMEOUT_S):
            raise ConfigurationError(
                'cmd_vel_timeout must be at most {}, got {}'.format(
                    MAX_CMD_VEL_TIMEOUT_S, self.cmd_vel_timeout_))

        # 1 = autonomous remote, 5 = shared remote. LUCI ignores a source
        # that has not been activated.
        self.input_source_ = self._declare(
            'input_source', INPUT_SOURCE_AUTONOMOUS_REMOTE)
        if self.input_source_ not in VALID_INPUT_SOURCES:
            raise ConfigurationError(
                'input_source must be one of {}, got {}'.format(
                    VALID_INPUT_SOURCES, self.input_source_))

        # Steady clock, so staleness works even with use_sim_time and no
        # /clock.
        self.clock_ = Clock(clock_type=ClockType.STEADY_TIME)
        self.latest_ = ZERO
        self.last_cmd_time_ = self.clock_.now()

        self.joystick_publisher_ = self.create_publisher(
            LuciJoystick, 'luci/remote_joystick', 10)
        self.subscription_ = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10)
        self.publish_timer_ = self.create_timer(
            PUBLISH_PERIOD_S, self.publish_callback, clock=self.clock_)

    def _declare(self, name, default):
        """Declare a read-only parameter; return it as a float or an int."""
        # Read-only (a set is rejected, not ignored); dynamic typing accepts
        # YAML integers for floats.
        self.declare_parameter(
            name, default,
            ParameterDescriptor(read_only=True, dynamic_typing=True))
        value = self.get_parameter(name).value
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ConfigurationError(
                '{} must be a number, got {!r}'.format(name, value)
            ) from error
        if not isinstance(default, int):
            return number
        if not math.isfinite(number) or number != int(number):
            raise ConfigurationError(
                '{} must be a whole number, got {}'.format(name, value))
        return int(number)

    def _is_stale(self):
        if self.cmd_vel_timeout_ <= 0.0:
            return False
        age = self.clock_.now() - self.last_cmd_time_
        return age > Duration(seconds=self.cmd_vel_timeout_)

    def listener_callback(self, msg):
        """Store the converted Twist as the latest command."""
        linear = msg.linear.x
        angular = msg.angular.z
        if not is_valid_input(linear, angular):
            self.get_logger().warning(
                'Ignoring invalid cmd_vel ({}, {}), publishing zero '
                'joystick'.format(linear, angular),
                throttle_duration_sec=1.0)
        self.latest_ = twist_to_joystick(linear, angular, self.params_)
        self.last_cmd_time_ = self.clock_.now()

    def publish_callback(self):
        """Publish the latest command, or zero if it is stale."""
        forward_back, left_right = ZERO if self._is_stale() else self.latest_
        msg = LuciJoystick()
        msg.forward_back = forward_back
        msg.left_right = left_right
        msg.input_source = self.input_source_
        self.joystick_publisher_.publish(msg)


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    try:
        joystick_converter = JoystickConverter()
    except ConfigurationError as error:
        get_logger('twist_to_joystick_node').fatal(
            'Invalid configuration: {}'.format(error))
        rclpy.shutdown()
        return 1

    try:
        rclpy.spin(joystick_converter)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        joystick_converter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

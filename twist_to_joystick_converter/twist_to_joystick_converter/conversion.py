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

"""Twist to joystick conversion, with no ROS dependency."""

from dataclasses import dataclass
import math
from typing import Tuple

# Joystick axes range over [-JOYSTICK_LIMIT, JOYSTICK_LIMIT].
JOYSTICK_LIMIT = 100
# Larger velocities (m/s or rad/s) are treated as corrupt.
MAX_INPUT = 100.0

# (forward_back, left_right) command that stops the chair.
ZERO = (0, 0)


@dataclass(frozen=True)
class ConversionParams:
    """
    Calibration constants for the Twist -> joystick mapping.

    Defaults are the Q500M values measured on drive profile 3 at speed
    setting 3 (see config/q500m.yaml).
    Raises ValueError for constants that cannot give a valid command:
    non-finite values, a speed or turn rate that is not above zero, a dead
    zone outside [0, JOYSTICK_LIMIT), js limits that are not ints within
    [-JOYSTICK_LIMIT, JOYSTICK_LIMIT], or overflow for inputs up to MAX_INPUT.
    """

    # Forward/back: the joystick value is a percentage of the profile's top
    # speed, so js = 100 * v / top speed (v in m/s). The chair's top speeds
    # differ between forward and reverse. Read them from the drive profile.
    max_forward_speed: float = 1.028  # m/s, 2.3 mph
    max_reverse_speed: float = 0.313  # m/s, 0.7 mph
    # Left/right: the chair does not turn below a dead zone of joystick units,
    # then speeds up in a straight line to max_turn_rate at full deflection:
    # |js| = dead_zone + (100 - dead_zone) * |w| / max_turn_rate (w in rad/s).
    max_turn_rate: float = 0.524  # rad/s at full deflection
    turn_dead_zone: float = 18.0  # joystick units
    # Joystick output limits
    js_min: int = -JOYSTICK_LIMIT
    js_max: int = JOYSTICK_LIMIT

    def forward_back(self, velocity: float) -> float:
        """Return the unclamped forward/back value (velocity in m/s)."""
        top = (self.max_forward_speed if velocity >= 0.0
               else self.max_reverse_speed)
        return JOYSTICK_LIMIT * velocity / top

    def left_right(self, velocity: float) -> float:
        """
        Return the unclamped left/right value (yaw rate in rad/s).

        A positive yaw (counter-clockwise) gives a negative value. Any
        nonzero yaw gives at least the dead zone, in the matching direction.
        """
        span = JOYSTICK_LIMIT - self.turn_dead_zone
        magnitude = (self.turn_dead_zone +
                     span * abs(velocity) / self.max_turn_rate)
        return -math.copysign(magnitude, velocity)

    def __post_init__(self):
        """Validate the constants."""
        if not all(map(math.isfinite, (
                self.max_forward_speed, self.max_reverse_speed,
                self.max_turn_rate, self.turn_dead_zone))):
            raise ValueError('The speeds, turn rate and dead zone must be '
                             'finite numbers')
        if min(self.max_forward_speed, self.max_reverse_speed,
               self.max_turn_rate) <= 0.0:
            raise ValueError('max_forward_speed, max_reverse_speed and '
                             'max_turn_rate must be greater than zero')
        if not 0.0 <= self.turn_dead_zone < JOYSTICK_LIMIT:
            raise ValueError(
                'turn_dead_zone must be at least 0 and below {}, got {}'.format(
                    JOYSTICK_LIMIT, self.turn_dead_zone))
        if not (isinstance(self.js_min, int) and
                isinstance(self.js_max, int)):
            raise ValueError('js_min and js_max must be ints')
        if not (-JOYSTICK_LIMIT <= self.js_min <= self.js_max <=
                JOYSTICK_LIMIT):
            raise ValueError(
                'Need -{0} <= js_min <= js_max <= {0}, got {1} and {2}'.format(
                    JOYSTICK_LIMIT, self.js_min, self.js_max))
        # Terms grow with the input's magnitude, so the extremes bound it.
        extremes = (self.forward_back(MAX_INPUT), self.forward_back(-MAX_INPUT),
                    self.left_right(MAX_INPUT), self.left_right(-MAX_INPUT))
        if not all(map(math.isfinite, extremes)):
            raise ValueError(
                'The constants overflow for inputs up to {}'.format(MAX_INPUT))


DEFAULT_PARAMS = ConversionParams()


def is_valid_input(linear_velocity: float, angular_velocity: float) -> bool:
    """Return True if both velocities are finite and within MAX_INPUT."""
    # NaN fails the comparison.
    return (abs(linear_velocity) <= MAX_INPUT and
            abs(angular_velocity) <= MAX_INPUT)


def twist_to_joystick(linear_velocity: float, angular_velocity: float,
                      params: ConversionParams = DEFAULT_PARAMS
                      ) -> Tuple[int, int]:
    """
    Convert linear and angular velocity to a joystick command.

    Zero maps to zero. Invalid input (see is_valid_input) also gives zero, so
    it cannot move the chair.

    :param linear_velocity: forward velocity in m/s.
    :param angular_velocity: yaw rate in rad/s.
    :param params: calibration constants.
    :return: (forward_back, left_right) ints in [js_min, js_max].
    """
    if not is_valid_input(linear_velocity, angular_velocity):
        return ZERO

    # The forward/back line is 0 at 0, so it needs no special case.
    forward_back = params.forward_back(linear_velocity)
    # The dead zone would map an exact 0 to a nonzero value.
    left_right = (0.0 if angular_velocity == 0.0
                  else params.left_right(angular_velocity))

    return (_clamp(round(forward_back), params),
            _clamp(round(left_right), params))


def _clamp(value: int, params: ConversionParams) -> int:
    return max(params.js_min, min(params.js_max, value))

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

    Defaults are the unvalidated Q500M values (see config/q500m.yaml).
    Raises ValueError for constants that cannot give a valid command:
    non-finite values, a zero slope, js limits that are not ints within
    [-JOYSTICK_LIMIT, JOYSTICK_LIMIT], or overflow for inputs up to MAX_INPUT.
    """

    # Forward/back: js = c3 * v^3 - c2 * v^2 + c1 * v   (v in m/s)
    forward_coeff_3: float = 208.6
    forward_coeff_2: float = 316.7
    forward_coeff_1: float = 203.3
    # Left/right: js = (w - angular_offset) / angular_slope   (w in rad/s)
    angular_offset: float = 0.0676
    angular_slope: float = -0.012212
    # Joystick output limits
    js_min: int = -JOYSTICK_LIMIT
    js_max: int = JOYSTICK_LIMIT

    def forward_back(self, velocity: float) -> float:
        """Return the unclamped forward/back value (velocity in m/s)."""
        return ((self.forward_coeff_3 * (velocity ** 3)) -
                (self.forward_coeff_2 * (velocity ** 2)) +
                (self.forward_coeff_1 * velocity))

    def left_right(self, velocity: float) -> float:
        """Return the unclamped left/right value (yaw rate in rad/s)."""
        return (velocity - self.angular_offset) / self.angular_slope

    def __post_init__(self):
        """Validate the constants."""
        if not all(map(math.isfinite, (
                self.forward_coeff_3, self.forward_coeff_2,
                self.forward_coeff_1, self.angular_offset,
                self.angular_slope))):
            raise ValueError('The coefficients, offset and slope must be '
                             'finite numbers')
        if self.angular_slope == 0.0:
            raise ValueError('angular_slope must not be zero')
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

    # The cubic is 0 at 0, so it needs no special case.
    forward_back = params.forward_back(linear_velocity)
    # The fit's offset would map an exact 0 to about +5.
    left_right = (0.0 if angular_velocity == 0.0
                  else params.left_right(angular_velocity))

    return (_clamp(int(forward_back), params),
            _clamp(int(left_right), params))


def _clamp(value: int, params: ConversionParams) -> int:
    return max(params.js_min, min(params.js_max, value))

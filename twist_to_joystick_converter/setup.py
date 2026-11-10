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

from glob import glob

from setuptools import setup

package_name = 'twist_to_joystick_converter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rupak Dasgupta',
    maintainer_email='rupak@luci.com',
    description='Converts geometry_msgs/Twist velocity commands (cmd_vel) '
                'into LuciJoystick commands for a LUCI-equipped wheelchair.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'twist_to_joystick_node = '
            'twist_to_joystick_converter.twist_to_joystick_node:main',
        ],
    },
)

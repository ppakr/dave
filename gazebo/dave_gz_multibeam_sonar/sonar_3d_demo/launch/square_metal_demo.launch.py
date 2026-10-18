# Copyright 2026
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dave_demos = get_package_share_directory("dave_demos")

    # Launch DAVE sim with the square_metal_demo world and 3d_sonar sensor.
    # Visualization (RViz + sonar aggregator) lives in viz.launch.py and is
    # started separately, once all the multibeam sensors have initialized.
    tank_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dave_demos, "launch", "dave_sensor.launch.py")
        ),
        launch_arguments={
            "namespace": "3d_sonar",
            "world_name": "square_metal_demo",
            "paused": "false",
            "x": "-0.95",
            "y": "0.0",
            "z": "0.3",
            "roll": "0.0",
            "pitch": "0.0",
            "yaw": "0.0",
            "debug": "true",
            "verbosity_level": "4",
        }.items(),
    )

    return LaunchDescription(
        [
            tank_sim,
        ]
    )

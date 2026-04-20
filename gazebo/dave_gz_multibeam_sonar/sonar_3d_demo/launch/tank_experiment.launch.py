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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_dave_demos = get_package_share_directory("dave_demos")
    pkg_sonar_3d_demo = get_package_share_directory("sonar_3d_demo")

    # Launch DAVE sim with the tank_experiment world and 3d_sonar sensor
    tank_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dave_demos, "launch", "dave_sensor.launch.py")
        ),
        launch_arguments={
            "namespace": "3d_sonar",
            "world_name": "tank_experiment",
            "paused": "false",
            "x": "-0.85",
            "y": "0.0",
            "z": "0.3",
            "roll": "0.0",
            "pitch": "0.0",
            "yaw": "0.0",
            "debug": "true",
            "verbosity_level": "4",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_sonar_3d_demo, "rviz", "tank_experiment.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            tank_sim,
            DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
            rviz,
        ]
    )

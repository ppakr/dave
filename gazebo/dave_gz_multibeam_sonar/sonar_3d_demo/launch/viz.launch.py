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

# Visualization stack for the sonar_3d_demo scenes. Start this *after* the
# sim launch (e.g. tank_scene.launch.py) has come up and all the sensors
# have initialized.
#
# Sensor-aware:
#   sensor:=sonar (default) -> runs sonar_aggregator + sonar RViz config
#   sensor:=lidar           -> skips the aggregator, loads the lidar RViz
#                              config (point cloud from /lidar_3d/lidar/points)

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


SENSOR_RVIZ = {
    "sonar": "square_metal_demo.rviz",
    "lidar": "square_metal_demo_lidar.rviz",
}


def launch_setup(context, *args, **kwargs):
    sensor = LaunchConfiguration("sensor").perform(context)
    rviz_config_arg = LaunchConfiguration("rviz_config").perform(context)
    use_rviz = LaunchConfiguration("rviz")

    if sensor not in SENSOR_RVIZ:
        raise RuntimeError(
            f"Unknown sensor '{sensor}'. Options: {sorted(SENSOR_RVIZ)}"
        )

    pkg_sonar_3d_demo = get_package_share_directory("sonar_3d_demo")
    rviz_config = rviz_config_arg or os.path.join(
        pkg_sonar_3d_demo, "rviz", SENSOR_RVIZ[sensor]
    )

    nodes = []
    if sensor == "sonar":
        nodes.append(
            Node(
                package="sonar_3d",
                executable="sonar_aggregator",
                output="screen",
            )
        )

    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
            output="screen",
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor",
                default_value="sonar",
                description="Which sensor's viz stack to bring up: 'sonar' or 'lidar'.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="Path to an RViz config file. Empty = pick default for sensor.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

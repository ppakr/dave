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

# Visualization stack for the sonar_3d_demo scenes: the sonar aggregator
# (fuses the per-beam multibeam clouds into /sensor/sonar_3d/pointcloud) plus
# RViz. Start this *after* the sim launch (e.g. square_metal_demo.launch.py)
# has come up and all the multibeam sensors have initialized, otherwise the
# aggregator has nothing to subscribe to.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_sonar_3d_demo = get_package_share_directory("sonar_3d_demo")
    default_rviz_config = os.path.join(
        pkg_sonar_3d_demo, "rviz", "square_metal_demo.rviz"
    )

    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("rviz")

    sonar_aggregator = Node(
        package="sonar_3d",
        executable="sonar_aggregator",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Path to the RViz config file.",
            ),
            sonar_aggregator,
            rviz,
        ]
    )

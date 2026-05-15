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

# Wetlab-tank scene launcher with sonar / lidar sensor switching. Spawns
# only the tank (from tank_experiment.world) plus a single sensor — no
# float or target. Mirrors the structure of tank_scene.launch.py for
# consistency.
#
# Usage:
#   ros2 launch sonar_3d_demo tank_experiment.launch.py                   # sonar
#   ros2 launch sonar_3d_demo tank_experiment.launch.py sensor:=lidar     # lidar
#
# The sonar aggregator is not started here — run it separately once the
# beams have initialised:
#   ros2 run sonar_3d sonar_aggregator    (sonar only; lidar publishes
#                                          /lidar_3d/lidar/points directly)

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

WORLD_NAME = "tank_experiment"

# Per-sensor configuration:
#   namespace   - dave_sensor_models/description/<namespace>/model.sdf
#   rviz_config - filename under sonar_3d_demo/rviz/
SENSOR_PROFILES = {
    "sonar": {
        "namespace": "3d_sonar",
        "rviz_config": "tank_experiment.rviz",
    },
    "lidar": {
        "namespace": "lidar_3d",
        "rviz_config": "tank_experiment_lidar.rviz",
    },
}


def _lidar_nodes():
    """Bridge + identity static TF for the 3D lidar.

    Matches the legacy tank_experiment_lidar.launch.py: identity TF
    `world -> lidar_3d/lidar_3d_base_link/gpu_lidar`. The accompanying
    RViz config sets Fixed Frame to the sensor frame, so points appear
    in the sensor's local coordinate system regardless of where the
    sensor is spawned in the world.
    """
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            ("/lidar", "/lidar_3d/lidar"),
            ("/lidar/points", "/lidar_3d/lidar/points"),
        ],
        output="screen",
    )
    tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_lidar",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            "lidar_3d/lidar_3d_base_link/gpu_lidar",
        ],
    )
    return [bridge, tf]


def launch_setup(context, *args, **kwargs):
    sensor = LaunchConfiguration("sensor").perform(context)

    if sensor not in SENSOR_PROFILES:
        raise RuntimeError(
            f"Unknown sensor '{sensor}'. Options: {sorted(SENSOR_PROFILES)}"
        )
    profile = SENSOR_PROFILES[sensor]
    pkg_sonar_3d_demo = get_package_share_directory("sonar_3d_demo")
    pkg_dave_demos = get_package_share_directory("dave_demos")

    tank_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dave_demos, "launch", "dave_sensor.launch.py")
        ),
        launch_arguments={
            "namespace": profile["namespace"],
            "world_name": WORLD_NAME,
            "paused": LaunchConfiguration("paused"),
            "debug": LaunchConfiguration("debug"),
            "verbosity_level": LaunchConfiguration("verbosity_level"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "roll": LaunchConfiguration("roll"),
            "pitch": LaunchConfiguration("pitch"),
            "yaw": LaunchConfiguration("yaw"),
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_sonar_3d_demo, "rviz", profile["rviz_config"]),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    nodes = [tank_sim, rviz]
    if sensor == "lidar":
        nodes += _lidar_nodes()

    print(f"[tank_experiment] sensor={sensor} (namespace={profile['namespace']})")
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor",
                default_value="sonar",
                description="Mounted sensor: 'sonar' (WaterLinked 3D) or 'lidar' (3D LiDAR)",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            # Sensor pose in the tank. Same default for both sensors so the
            # two scenes are directly comparable.
            DeclareLaunchArgument("x", default_value="-0.85"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.5"),
            DeclareLaunchArgument("roll", default_value="0.0"),
            DeclareLaunchArgument("pitch", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("debug", default_value="true"),
            DeclareLaunchArgument("verbosity_level", default_value="4"),
            OpaqueFunction(function=launch_setup),
        ]
    )

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

# 3D LiDAR counterpart to tank_experiment.launch.py. Spawns the
# dave_sensor_models/lidar_3d sensor at the same pose as the 3D sonar
# in the WaterLinked wetlab tank world, so the two clouds can be
# compared side by side against the real-world recording.

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

    # Same world, same spawn pose, same orientation as the sonar — only the
    # sensor model differs. Namespace 'lidar_3d' picks up
    # dave_sensor_models/description/lidar_3d/model.sdf.
    tank_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dave_demos, "launch", "dave_sensor.launch.py")
        ),
        launch_arguments={
            "namespace": "lidar_3d",
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

    # The lidar_3d model.sdf has no companion sensor_config.py under
    # dave_sensor_models/config/, so we have to bridge the gz topics ourselves.
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

    # Identity TF from world to the gz-scoped sensor frame, mirroring what the
    # sonar config does for 3d_sonar/base_link. Fixed frame in RViz is set to
    # this sensor frame, so points display in their own coordinate system.
    tf_lidar = Node(
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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_sonar_3d_demo, "rviz", "tank_experiment_lidar.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
            tank_sim,
            bridge,
            tf_lidar,
            rviz,
        ]
    )

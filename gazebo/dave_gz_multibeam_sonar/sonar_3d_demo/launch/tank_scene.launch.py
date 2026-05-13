# Parametric launcher for the tank+float scene.
#
# Loads the constant scene (tank, float, sonar, lights) from
# tank_with_float.world via dave_sensor.launch.py, then spawns one target
# from ~/blender_models/<target>/ using the per-target metadata in
# config/targets.yaml.
#
# Usage:
#   ros2 launch sonar_3d_demo tank_scene.launch.py target:=brick
#   ros2 launch sonar_3d_demo tank_scene.launch.py target:=triangle_wood yaw:=0.5
#   ros2 launch sonar_3d_demo tank_scene.launch.py target:=square_metal string_length:=0.50

import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BLENDER_MODELS_DIR = os.path.expanduser("~/blender_models")
WORLD_NAME = "tank_with_float"

# Sonar pose in the tank, matching the existing square_metal_demo.launch.py.
SONAR_POSE = {"x": "-0.95", "y": "0.0", "z": "0.3",
              "roll": "0.0", "pitch": "0.0", "yaw": "0.0"}


def _resolve(arg_value, fallback):
    """Return arg_value if non-empty, else fallback. Both as float."""
    s = str(arg_value).strip()
    return float(s) if s else float(fallback)


def launch_setup(context, *args, **kwargs):
    target = LaunchConfiguration("target").perform(context)
    string_length_arg = LaunchConfiguration("string_length").perform(context)
    yaw_arg = LaunchConfiguration("yaw").perform(context)
    x_arg = LaunchConfiguration("x").perform(context)
    y_arg = LaunchConfiguration("y").perform(context)

    config_path = os.path.join(
        get_package_share_directory("sonar_3d_demo"), "config", "targets.yaml"
    )
    with open(config_path) as f:
        manifest = yaml.safe_load(f)

    if target not in manifest["targets"]:
        raise RuntimeError(
            f"Unknown target '{target}'. Available: {sorted(manifest['targets'])}"
        )
    scene = manifest["scene"]
    spec = manifest["targets"][target]

    string_length = _resolve(string_length_arg, scene["default_string_length"])
    yaw = _resolve(yaw_arg, spec.get("yaw", 0.0))
    x = _resolve(x_arg, scene["default_xy"][0])
    y = _resolve(y_arg, scene["default_xy"][1])
    z = scene["float_top_z"] - string_length - spec["top_z_offset"]

    sdf_path = os.path.join(BLENDER_MODELS_DIR, target, "model.sdf")
    if not os.path.isfile(sdf_path):
        raise RuntimeError(f"Target SDF not found: {sdf_path}")

    print(
        f"[tank_scene] target={target} pose=({x:.3f}, {y:.3f}, {z:.3f}) yaw={yaw:.4f} "
        f"(float_top_z={scene['float_top_z']}, string={string_length:.3f}, "
        f"top_z_offset={spec['top_z_offset']})"
    )

    tank_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dave_demos"), "launch",
                "dave_sensor.launch.py"
            )
        ),
        launch_arguments={
            "namespace": "3d_sonar",
            "world_name": WORLD_NAME,
            "paused": LaunchConfiguration("paused"),
            "debug": LaunchConfiguration("debug"),
            "verbosity_level": LaunchConfiguration("verbosity_level"),
            **SONAR_POSE,
        }.items(),
    )

    target_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "default",
            "-name", target,
            "-file", sdf_path,
            "-x", f"{x}",
            "-y", f"{y}",
            "-z", f"{z}",
            "-Y", f"{yaw}",
        ],
        output="both",
        parameters=[{"use_sim_time": True}],
    )

    return [tank_sim, target_spawner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "target",
            description="Target model folder name in ~/blender_models (e.g. brick, square_metal)",
        ),
        DeclareLaunchArgument(
            "string_length", default_value="",
            description="m, top of target below float top. Empty = YAML default.",
        ),
        DeclareLaunchArgument(
            "yaw", default_value="",
            description="rad, override target yaw. Empty = per-target YAML default.",
        ),
        DeclareLaunchArgument(
            "x", default_value="",
            description="m, override target x. Empty = scene.default_xy[0].",
        ),
        DeclareLaunchArgument(
            "y", default_value="",
            description="m, override target y. Empty = scene.default_xy[1].",
        ),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("debug", default_value="true"),
        DeclareLaunchArgument("verbosity_level", default_value="4"),
        OpaqueFunction(function=launch_setup),
    ])

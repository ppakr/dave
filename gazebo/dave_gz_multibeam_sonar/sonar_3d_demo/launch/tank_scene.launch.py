# Parametric launcher for the tank+float scene.
#
# Drives the whole scene from config/targets.yaml:
#   - scene.float   -> float pose (spawned at launch time)
#   - scene.sonar_3d -> sonar pose (passed to dave_sensor.launch.py)
#   - targets.<name> -> target pose (spawned at launch time)
#
# The world file (tank_with_float.world) only carries the tank, lights, and
# GUI plugins; everything pose-related lives in the YAML.
#
# Usage:
#   ros2 launch sonar_3d_demo tank_scene.launch.py target:=brick
#   ros2 launch sonar_3d_demo tank_scene.launch.py target:=triangle_wood yaw:=0.5
#   ros2 launch sonar_3d_demo tank_scene.launch.py target:=square_metal z:=0.25

import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BLENDER_MODELS_DIR = os.path.expanduser("~/blender_models")
WORLD_NAME = "tank_with_float"
POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def _resolve(arg_value, fallback):
    """Return arg_value if non-empty, else fallback. Both coerced to float."""
    s = str(arg_value).strip()
    return float(s) if s else float(fallback)


def _spawn_node(name, sdf_path, pose):
    """Build a ros_gz_sim create Node for the given model + pose dict."""
    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{name}",
        arguments=[
            "-world", "default",
            "-name", name,
            "-file", sdf_path,
            "-x", f"{pose['x']}",
            "-y", f"{pose['y']}",
            "-z", f"{pose['z']}",
            "-R", f"{pose['roll']}",
            "-P", f"{pose['pitch']}",
            "-Y", f"{pose['yaw']}",
        ],
        output="both",
        parameters=[{"use_sim_time": True}],
    )


def launch_setup(context, *args, **kwargs):
    target = LaunchConfiguration("target").perform(context)

    # CLI overrides for the target pose (empty string -> use YAML)
    pose_overrides = {
        k: LaunchConfiguration(k).perform(context) for k in POSE_KEYS
    }

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
    float_pose = scene["float"]
    sonar_pose = scene["sonar_3d"]
    target_yaml = manifest["targets"][target]

    # Resolve target pose: CLI override beats YAML.
    target_pose = {
        k: _resolve(pose_overrides[k], target_yaml[k]) for k in POSE_KEYS
    }

    float_sdf = os.path.join(BLENDER_MODELS_DIR, "float", "model.sdf")
    target_sdf = os.path.join(BLENDER_MODELS_DIR, target, "model.sdf")
    for path in (float_sdf, target_sdf):
        if not os.path.isfile(path):
            raise RuntimeError(f"SDF not found: {path}")

    print(
        f"[tank_scene] target={target} "
        f"pose=({target_pose['x']:.3f}, {target_pose['y']:.3f}, {target_pose['z']:.3f}) "
        f"rpy=({target_pose['roll']:.4f}, {target_pose['pitch']:.4f}, {target_pose['yaw']:.4f})"
    )

    tank_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dave_demos"),
                "launch",
                "dave_sensor.launch.py",
            )
        ),
        launch_arguments={
            "namespace": "3d_sonar",
            "world_name": WORLD_NAME,
            "paused": LaunchConfiguration("paused"),
            "debug": LaunchConfiguration("debug"),
            "verbosity_level": LaunchConfiguration("verbosity_level"),
            "x": str(sonar_pose["x"]),
            "y": str(sonar_pose["y"]),
            "z": str(sonar_pose["z"]),
            "roll": str(sonar_pose["roll"]),
            "pitch": str(sonar_pose["pitch"]),
            "yaw": str(sonar_pose["yaw"]),
        }.items(),
    )

    float_spawner = _spawn_node("float", float_sdf, float_pose)
    target_spawner = _spawn_node(target, target_sdf, target_pose)

    return [tank_sim, float_spawner, target_spawner]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "target",
            description="Target model folder name in ~/blender_models (e.g. brick, square_metal)",
        ),
    ]
    # CLI pose overrides for the target. Empty string -> use YAML value.
    for k in POSE_KEYS:
        args.append(
            DeclareLaunchArgument(
                k,
                default_value="",
                description=f"Override target {k} (m or rad). Empty = YAML value.",
            )
        )
    args += [
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("debug", default_value="true"),
        DeclareLaunchArgument("verbosity_level", default_value="4"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])

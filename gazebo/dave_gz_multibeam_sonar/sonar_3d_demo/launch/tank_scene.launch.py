# Parametric launcher for the tank+float scene.
#
# Drives the whole scene from config/targets.yaml:
#   - scene.float    -> float pose (spawned at launch time)
#   - scene.sonar_3d -> sonar pose  (when sensor:=sonar)
#   - scene.lidar_3d -> lidar pose  (when sensor:=lidar)
#   - targets.<name> -> target pose (spawned at launch time)
#
# The world file (tank_with_float.world) only carries the tank, lights, and
# GUI plugins; everything pose-related lives in the YAML.
#
# Usage:
#   ros2 launch sonar_3d_demo tank_scene.launch.py target:=brick
#   ros2 launch sonar_3d_demo tank_scene.launch.py target:=brick sensor:=lidar
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

# Map the user-facing `sensor` arg to (namespace, YAML key).
# The namespace is what dave_sensor.launch.py uses to look up the model
# under dave_sensor_models/description/<namespace>/model.sdf, and what gets
# prefixed onto Gazebo's published frames.
SENSOR_PROFILES = {
    "sonar": {"namespace": "3d_sonar", "yaml_key": "sonar_3d"},
    "lidar": {"namespace": "lidar_3d", "yaml_key": "lidar_3d"},
}


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


def _lidar_nodes(pose):
    """Bridge + static TF for the 3D lidar.

    Mirrors lidar_3d_demo.launch.py, but pins the static TF to the YAML
    lidar pose so RViz sees the lidar where it actually is in the tank.
    """
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
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
        arguments=[
            "--x", f"{pose['x']}",
            "--y", f"{pose['y']}",
            "--z", f"{pose['z']}",
            "--roll", f"{pose['roll']}",
            "--pitch", f"{pose['pitch']}",
            "--yaw", f"{pose['yaw']}",
            "--frame-id", "world",
            "--child-frame-id", "lidar_3d/lidar_3d_base_link/gpu_lidar",
        ],
    )
    return [bridge, tf]


def launch_setup(context, *args, **kwargs):
    target = LaunchConfiguration("target").perform(context)
    sensor = LaunchConfiguration("sensor").perform(context)

    if sensor not in SENSOR_PROFILES:
        raise RuntimeError(
            f"Unknown sensor '{sensor}'. Options: {sorted(SENSOR_PROFILES)}"
        )
    profile = SENSOR_PROFILES[sensor]

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
    if profile["yaml_key"] not in scene:
        raise RuntimeError(
            f"Missing 'scene.{profile['yaml_key']}' in targets.yaml "
            f"(needed for sensor:={sensor})"
        )
    sensor_pose = scene[profile["yaml_key"]]
    float_pose = scene["float"]
    target_yaml = manifest["targets"][target]

    target_pose = {
        k: _resolve(pose_overrides[k], target_yaml[k]) for k in POSE_KEYS
    }

    float_sdf = os.path.join(BLENDER_MODELS_DIR, "float", "model.sdf")
    target_sdf = os.path.join(BLENDER_MODELS_DIR, target, "model.sdf")
    for path in (float_sdf, target_sdf):
        if not os.path.isfile(path):
            raise RuntimeError(f"SDF not found: {path}")

    print(
        f"[tank_scene] sensor={sensor} (namespace={profile['namespace']}) "
        f"target={target} "
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
            "namespace": profile["namespace"],
            "world_name": WORLD_NAME,
            "paused": LaunchConfiguration("paused"),
            "debug": LaunchConfiguration("debug"),
            "verbosity_level": LaunchConfiguration("verbosity_level"),
            "x": str(sensor_pose["x"]),
            "y": str(sensor_pose["y"]),
            "z": str(sensor_pose["z"]),
            "roll": str(sensor_pose["roll"]),
            "pitch": str(sensor_pose["pitch"]),
            "yaw": str(sensor_pose["yaw"]),
        }.items(),
    )

    float_spawner = _spawn_node("float", float_sdf, float_pose)
    target_spawner = _spawn_node(target, target_sdf, target_pose)

    nodes = [tank_sim, float_spawner, target_spawner]
    if sensor == "lidar":
        nodes += _lidar_nodes(sensor_pose)
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "target",
            description="Target model folder name in ~/blender_models (e.g. brick, square_metal)",
        ),
        DeclareLaunchArgument(
            "sensor",
            default_value="sonar",
            description="Which sensor to mount: 'sonar' (WaterLinked 3D sonar) or 'lidar' (3D LiDAR)",
        ),
    ]
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

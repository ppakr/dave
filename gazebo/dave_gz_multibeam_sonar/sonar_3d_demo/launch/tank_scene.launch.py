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
# pose_source argument
# --------------------
# yaml            Use the position+rotation from targets.yaml (legacy behaviour).
# real_median     (default) Look up the target's median position in
#                 <poses_dir>/summary_static.csv (extracted by
#                 sonar_camera_logger/scripts/extract_object_poses.py), and
#                 spawn at sim_world_T_sonar ⊕ sonar_T_object. Rotation still
#                 comes from the YAML — the CSV's quaternion mostly reflects
#                 floater spin, not the intended target orientation.
#                 Falls back silently to YAML if no row matches the target.
# real_trajectory Same as real_median for spawn, but also exposes the per-bag
#                 timeseries CSV to a downstream playback node (task wired in
#                 a separate launch helper).
#
# Usage:
#   ros2 launch sonar_3d_demo tank_scene.launch.py target:=brick
#   ros2 launch sonar_3d_demo tank_scene.launch.py target:=square_petg sensor:=lidar
#   ros2 launch sonar_3d_demo tank_scene.launch.py target:=square_metal z:=0.25
#   ros2 launch sonar_3d_demo tank_scene.launch.py target:=circle_petg pose_source:=yaml

import csv
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BLENDER_MODELS_DIR = os.path.expanduser("~/blender_models")
WORLD_NAME = "tank_with_float"
POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")

DEFAULT_POSES_DIR = (
    "/media/aki/2C76C6780AEDB4DB1/wl_wetlab_apr22_processed/object_poses"
)
POSE_SOURCES = ("yaml", "real_median", "real_trajectory")

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


def _lookup_real_median(target: str, poses_dir: str):
    """Return (x, y, z) of `target` in the sonar frame from summary_static.csv.

    Returns None if the CSV is missing, has no row for this target, or any
    row value is unparseable. The caller is expected to fall back to the YAML.
    """
    summary = os.path.join(poses_dir, "summary_static.csv")
    if not os.path.isfile(summary):
        return None
    with open(summary) as f:
        for row in csv.DictReader(f):
            if row.get("target") != target:
                continue
            try:
                return (
                    float(row["med_x"]),
                    float(row["med_y"]),
                    float(row["med_z"]),
                )
            except (KeyError, ValueError):
                return None
    return None


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
    pose_source = LaunchConfiguration("pose_source").perform(context)
    poses_dir = LaunchConfiguration("poses_dir").perform(context)

    if sensor not in SENSOR_PROFILES:
        raise RuntimeError(
            f"Unknown sensor '{sensor}'. Options: {sorted(SENSOR_PROFILES)}"
        )
    if pose_source not in POSE_SOURCES:
        raise RuntimeError(
            f"Unknown pose_source '{pose_source}'. Options: {list(POSE_SOURCES)}"
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

    # YAML baseline pose (rotations always come from here — see module docstring).
    target_pose = {
        k: _resolve(pose_overrides[k], target_yaml[k]) for k in POSE_KEYS
    }

    # Optionally overwrite XYZ with the real median pose. We compose
    # sim_world_T_object = sim_world_T_sonar (translation) + sonar_T_object (CSV),
    # which assumes sensor_pose has identity rotation — true for the current
    # targets.yaml. CLI overrides on x/y/z (non-empty pose_overrides) still win.
    real_xyz_used = False
    if pose_source in ("real_median", "real_trajectory"):
        real_xyz = _lookup_real_median(target, poses_dir)
        if real_xyz is not None:
            sx, sy, sz = sensor_pose["x"], sensor_pose["y"], sensor_pose["z"]
            world_xyz = (sx + real_xyz[0], sy + real_xyz[1], sz + real_xyz[2])
            for k, val in zip(("x", "y", "z"), world_xyz):
                if not str(pose_overrides[k]).strip():
                    target_pose[k] = float(val)
            real_xyz_used = True
        else:
            print(
                f"[tank_scene] pose_source={pose_source} but no row for "
                f"target='{target}' in {poses_dir}/summary_static.csv — "
                f"falling back to YAML."
            )

    float_sdf = os.path.join(BLENDER_MODELS_DIR, "float", "model.sdf")
    target_sdf = os.path.join(BLENDER_MODELS_DIR, target, "model.sdf")
    for path in (float_sdf, target_sdf):
        if not os.path.isfile(path):
            raise RuntimeError(f"SDF not found: {path}")

    print(
        f"[tank_scene] sensor={sensor} (namespace={profile['namespace']}) "
        f"target={target} pose_source={pose_source}"
        f"{' (CSV)' if real_xyz_used else ' (YAML)'}\n"
        f"            pose=({target_pose['x']:.3f}, {target_pose['y']:.3f}, "
        f"{target_pose['z']:.3f}) "
        f"rpy=({target_pose['roll']:.4f}, {target_pose['pitch']:.4f}, "
        f"{target_pose['yaw']:.4f})"
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

    # Stagger the spawns and push them past Gazebo startup. The GUI and the
    # server-side rendering sensors (gpu_lidar / cameras) use *separate*
    # gz-rendering scenes; a model created in the same step the Sensors system
    # is still building its scene can land in the GUI scene but be missed by the
    # sensor scene -> visible in Gazebo but invisible to the lidar. Spawning
    # after the sensor scene is up, one model per timer, avoids that race.
    # Bump the periods if Gazebo is slow to initialize on your machine.
    float_spawner = TimerAction(period=5.0, actions=[float_spawner])
    target_spawner = TimerAction(period=7.0, actions=[target_spawner])

    nodes = [tank_sim, float_spawner, target_spawner]
    if sensor == "lidar":
        nodes += _lidar_nodes(sensor_pose)

    if pose_source == "real_trajectory":
        bag = LaunchConfiguration("bag").perform(context).strip()
        if not bag:
            raise RuntimeError(
                "pose_source=real_trajectory requires bag:=<bag_name> "
                "(e.g. apr22_petg_circle_moving)."
            )
        csv_path = os.path.join(poses_dir, f"{bag}.csv")
        if not os.path.isfile(csv_path):
            raise RuntimeError(f"Trajectory CSV not found: {csv_path}")
        player = Node(
            package="sonar_3d_demo",
            executable="play_object_trajectory.py",
            name="play_object_trajectory",
            arguments=[
                "--csv", csv_path,
                "--model_name", target,
                "--world", "default",
                "--sensor_x", str(sensor_pose["x"]),
                "--sensor_y", str(sensor_pose["y"]),
                "--sensor_z", str(sensor_pose["z"]),
                "--rate", LaunchConfiguration("rate").perform(context),
                "--startup_delay_s", "8.0",
            ],
            output="screen",
        )
        # Start after the spawner timer (period=7s).
        nodes.append(TimerAction(period=9.0, actions=[player]))

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
        DeclareLaunchArgument(
            "pose_source",
            default_value="real_median",
            description=(
                "Where the target pose comes from: 'yaml' (config/targets.yaml), "
                "'real_median' (median XYZ from summary_static.csv, YAML rotation), "
                "or 'real_trajectory' (median XYZ at spawn, playback via "
                "trajectory_player.launch.py)."
            ),
        ),
        DeclareLaunchArgument(
            "poses_dir",
            default_value=DEFAULT_POSES_DIR,
            description="Directory containing summary_static.csv and per-bag CSVs.",
        ),
        DeclareLaunchArgument(
            "bag",
            default_value="",
            description=(
                "When pose_source=real_trajectory, which per-bag CSV (basename "
                "without .csv) to play back, e.g. apr22_petg_circle_moving."
            ),
        ),
        DeclareLaunchArgument(
            "rate",
            default_value="10.0",
            description="Trajectory playback rate (Hz). Higher = more gz service calls.",
        ),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])

# 3D Sonar for Gazebo

Aggregator node + supporting tooling for the WaterLinked 3D sonar.
The sensor is built as a stack of 64 narrow multibeam sonars, each with a
different pitch (`-18.9°` to `+18.9°` in `0.60°` steps). `sonar_aggregator`
subscribes to all 64 beams, projects each return into 3D using the per-beam
pitch, and republishes a single combined point cloud.

## Prerequisites

- NVIDIA GPU with CUDA Toolkit (the underlying `multibeam_sonar` plugin is
  CUDA-only and is silently skipped at build time on machines without it).
- ROS 2 Jazzy + Gazebo Harmonic.
- The workspace must be built and sourced:

  ```bash
  colcon build && source install/setup.bash
  ```

## Running

Pick a scene, start its sim launch, wait for the beams to initialize, then
bring up visualization.

### Option A — DAVE multibeam sonar test world

The original DAVE sonar scene (sonar pointed at sample objects in
`dave_multibeam_sonar.world`):

```bash
ros2 launch dave_multibeam_sonar_demo 3d_sonar_demo.launch.py
```

This launch opens RViz itself; `rviz:=false` skips it. Start the aggregator
separately (see below).

### Option B — WaterLinked wetlab tank

The validation scene that mirrors the real-tank recording setup
(`tank_experiment.world`, with the `WL_wetlab_tank` model fetched from
Gazebo Fuel on first launch). Like scenes C and D, this is **sim only** —
RViz and the aggregator come from `viz.launch.py`:

```bash
# terminal 1 — sim (sonar by default, sensor:=lidar for lidar)
ros2 launch sonar_3d_demo tank_experiment.launch.py
ros2 launch sonar_3d_demo tank_experiment.launch.py sensor:=lidar

# terminal 2 — once the sensors have initialized; use the tank-specific
# RViz layouts via rviz_config (viz.launch.py defaults to the
# square_metal layouts otherwise)
ros2 launch sonar_3d_demo viz.launch.py \
    rviz_config:=$(ros2 pkg prefix sonar_3d_demo)/share/sonar_3d_demo/rviz/tank_experiment.rviz
ros2 launch sonar_3d_demo viz.launch.py sensor:=lidar \
    rviz_config:=$(ros2 pkg prefix sonar_3d_demo)/share/sonar_3d_demo/rviz/tank_experiment_lidar.rviz
```

### Option C — square metal target in the wetlab tank

A variant of the wetlab-tank scene with a `square_metal` model placed 1.5 m
straight ahead of the sonar and 0.75 m below the water surface
(`square_metal_demo.world`; the `square_metal` model lives in
`dave_object_models/description/`). Unlike A and B, this launch is **sim
only** — RViz and the aggregator come from `viz.launch.py`:

```bash
# terminal 1 — sim
ros2 launch sonar_3d_demo square_metal_demo.launch.py

# terminal 2 — once the beams have initialized (see below)
ros2 launch sonar_3d_demo viz.launch.py
```

`viz.launch.py` runs `sonar_aggregator` and RViz together. Arguments:
`rviz:=false` to skip RViz, `rviz_config:=/path/to/foo.rviz` to use a
different layout (defaults to `sonar_3d_demo/rviz/square_metal_demo.rviz`).

### Option D — parametric tank+float scene with selectable target and sensor

A variant of scene C built for swapping between the full target library and
between sonar / lidar sensors without editing the world. The world
(`tank_with_float.world`) only holds the tank, lights, and GUI plugins.
Float, sensor, and target are spawned at launch time from
`sonar_3d_demo/config/targets.yaml` — that YAML is the single source of
truth for every pose in the scene. Like scene C, this is **sim only** —
RViz comes from `viz.launch.py`.

```bash
# terminal 1 — sim, sensor + target selectable
ros2 launch sonar_3d_demo tank_scene.launch.py target:=square_metal
ros2 launch sonar_3d_demo tank_scene.launch.py target:=triangle_wood
ros2 launch sonar_3d_demo tank_scene.launch.py target:=brick z:=0.25
ros2 launch sonar_3d_demo tank_scene.launch.py target:=square_metal sensor:=lidar

# terminal 2 — once the sensors have initialized
ros2 launch sonar_3d_demo viz.launch.py                  # sonar (default)
ros2 launch sonar_3d_demo viz.launch.py sensor:=lidar    # lidar
```

Available targets (folder names under `~/blender_models/`, also keys in
`config/targets.yaml`): `brick`, `circle_metal`, `circle_petg`,
`circle_wood`, `metal_board`, `square_metal`, `square_petg`, `square_wood`,
`triangle_metal`, `triangle_petg`, `triangle_wood`.

#### YAML structure

```yaml
scene:
  float:    {x: ..., y: ..., z: ..., roll: ..., pitch: ..., yaw: ...}
  sonar_3d: {x: ..., y: ..., z: ..., roll: ..., pitch: ..., yaw: ...}
  lidar_3d: {x: ..., y: ..., z: ..., roll: ..., pitch: ..., yaw: ...}

targets:
  <model_name>:
    x: ...   y: ...   z: ...
    roll: ...   pitch: ...   yaw: ...
```

All poses are 6-DOF (m / rad) in the Gazebo world frame. The launch reads
`scene.float` for the float spawn, `scene.sonar_3d` or `scene.lidar_3d`
(depending on `sensor`) for the sensor spawn, and `targets.<name>` for the
target spawn.

#### Launch arguments

`tank_scene.launch.py`:

| Arg | Default | Meaning |
|---|---|---|
| `target` | *(required)* | Folder name under `~/blender_models/` |
| `sensor` | `sonar` | Mounted sensor: `sonar` (WaterLinked 3D) or `lidar` (3D LiDAR) |
| `x`, `y`, `z`, `roll`, `pitch`, `yaw` | *empty → YAML value* | CLI overrides for the **target** pose (not the sensor) |
| `paused` | `false` | Start the sim paused |
| `debug`, `verbosity_level` | `true`, `4` | Forwarded to `dave_sensor.launch.py` |

`viz.launch.py`:

| Arg | Default | Meaning |
|---|---|---|
| `sensor` | `sonar` | Picks the RViz config and toggles the sonar aggregator |
| `rviz` | `true` | Open RViz |
| `rviz_config` | *empty → sensor default* | Override the RViz `.rviz` path |

Sensor-specific viz behaviour:

- `sensor:=sonar` — loads `rviz/square_metal_demo.rviz` and starts
  `sonar_aggregator` (which fuses the 64 multibeam clouds into
  `/sensor/sonar_3d/pointcloud`).
- `sensor:=lidar` — loads `rviz/square_metal_demo_lidar.rviz` (fixed frame
  `world`, point cloud on `/lidar_3d/lidar/points`); aggregator is skipped.
  In this mode `tank_scene.launch.py` also runs a `ros_gz_bridge` for the
  lidar topics and a `static_transform_publisher` pinning
  `lidar_3d/lidar_3d_base_link/gpu_lidar` to the YAML lidar pose.

#### Editing the scene

- **Move the float / sonar / lidar:** edit `scene.float`, `scene.sonar_3d`,
  or `scene.lidar_3d` in `config/targets.yaml` and relaunch.
- **Move a target permanently:** edit `targets.<name>` in the same file.
- **Move a target for one run:** pass any of `x:=`, `y:=`, `z:=`, `roll:=`,
  `pitch:=`, `yaw:=` on the command line — those override the YAML value
  for that target only.
- **Add a new target model:** drop a `model.sdf` (+ `meshes/`) into
  `~/blender_models/<new_name>/`, then add a `<new_name>:` block under
  `targets:` in the YAML. No code changes required.

### Starting the aggregator

The sim launch spawns the 64 beams asynchronously. **Wait until all beams
have initialized** before starting the aggregator, or some beams will be
missing from the first cloud — watch the Gazebo log until the
`Initializing [3d_sonar::base_link::sonar_3d_<i>] sensor` lines stop
(~20–30 s), or check `ros2 topic hz` on a high-numbered beam.

- For scene A, run it directly:

  ```bash
  ros2 run sonar_3d sonar_aggregator
  ```

- For scenes B, C, and D it is already part of `viz.launch.py` (start
  that launch only after the beams are up). In scenes B and D,
  `sensor:=lidar` skips the aggregator entirely — the lidar publishes
  its own point cloud directly on `/lidar_3d/lidar/points`.

### Gotcha — clean up before re-launching

`gz sim` ignores `SIGTERM`, so a `Ctrl+C`'d run can leave an orphaned
`gz sim` (plus stray `static_transform_publisher` / `rviz2` /
`parameter_bridge` processes) behind. The next launch then collides with
them: Gazebo hangs or exits, `ros2 launch` tears everything down with
`SIGINT` → `SIGTERM`, and RViz never shows. Hard-kill the stragglers
between runs:

```bash
pkill -9 -f "gz sim"; pkill -9 -f "/usr/bin/gz"; pkill -9 -f ruby
pkill -9 -f rviz2; pkill -9 -f parameter_bridge; pkill -9 -f static_transform_publisher
sleep 2
```

If launches still die after a clean kill, suspect the GPU/EGL stack
(`libEGL warning: egl: failed to create dri2 screen` in the log) — try
`LIBGL_ALWAYS_SOFTWARE=1` or `headless:=true` to isolate it.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/sensor/sonar_3d/beam_<i>/sonar_image_raw_<i>` (i = 0..63) | `marine_acoustic_msgs/ProjectedSonarImage` | aggregator subscribes |
| `/sensor/sonar_3d/pointcloud` | `sensor_msgs/PointCloud2` | aggregator publishes |

The topic prefix `/sensor/sonar_3d/...` is hardcoded in the aggregator and
does not depend on the launch-file `namespace` argument.

## Sensor geometry

The 64-beam stack is generated by `generate_3d_sonar.py`, which writes the
multi-sensor SDF block. The per-beam pitch formula
(`-20.0 + 1.10 + i * 0.60` degrees) is duplicated in `Sonar3D.cc` as
`sonar_pitch_rad(i)` — keep the two in sync if you change the geometry.

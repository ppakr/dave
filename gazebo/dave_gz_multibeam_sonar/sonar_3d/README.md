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
Gazebo Fuel on first launch):

```bash
ros2 launch sonar_3d_demo tank_experiment.launch.py
```

This launch also opens RViz itself; `rviz:=false` skips it.

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

### Option D — parametric tank+float scene with selectable target

A variant of scene C built for swapping between the full target library
without editing the world file. The scene (tank, float, sonar, lights) is
held constant in `tank_with_float.world`; the target is spawned at launch
time from `~/blender_models/<target>/` using the per-target metadata in
`sonar_3d_demo/config/targets.yaml`. Like scene C, this is **sim only** —
RViz and the aggregator come from `viz.launch.py`.

```bash
# terminal 1 — sim, target selectable
ros2 launch sonar_3d_demo tank_scene.launch.py target:=square_metal
ros2 launch sonar_3d_demo tank_scene.launch.py target:=triangle_wood
ros2 launch sonar_3d_demo tank_scene.launch.py target:=brick string_length:=0.50

# terminal 2 — once the beams have initialized
ros2 launch sonar_3d_demo viz.launch.py
```

Available targets (folder names under `~/blender_models/`, also keys in
`config/targets.yaml`): `brick`, `circle_metal`, `circle_petg`,
`circle_wood`, `metal_board`, `square_metal`, `square_petg`, `square_wood`,
`triangle_metal`, `triangle_petg`, `triangle_wood`.

Launch arguments:

| Arg | Default | Meaning |
|---|---|---|
| `target` | *(required)* | Folder name under `~/blender_models/` |
| `string_length` | YAML `scene.default_string_length` (`0.70 m`) | Distance from float-top to target-top |
| `yaw` | YAML `targets.<target>.yaw` | Rotation about Z, rad |
| `x`, `y` | YAML `scene.default_xy` (`(0.65, 0.0)`) | Horizontal anchor under the float |
| `paused` | `false` | Start the sim paused |
| `debug`, `verbosity_level` | `true`, `4` | Forwarded to `dave_sensor.launch.py` |

Placement formula (computed in the launch file):

```
target_z_origin = scene.float_top_z - string_length - target.top_z_offset
```

where `top_z_offset` is the height of the target's top above its model
origin (measured from the GLB bounding box; pre-computed in the YAML and
**must be re-measured if you re-export a CAD model**). The `float_top_z`
constant in the YAML must stay in sync with the float pose in
`tank_with_float.world` — if you move the float in the world, update the
YAML.

### Starting the aggregator

The sim launch spawns the 64 beams asynchronously. **Wait until all beams
have initialized** before starting the aggregator, or some beams will be
missing from the first cloud — watch the Gazebo log until the
`Initializing [3d_sonar::base_link::sonar_3d_<i>] sensor` lines stop
(~20–30 s), or check `ros2 topic hz` on a high-numbered beam.

- For scenes A and B, run it directly:

  ```bash
  ros2 run sonar_3d sonar_aggregator
  ```

- For scenes C and D it is already part of `viz.launch.py` (start that
  launch only after the beams are up).

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

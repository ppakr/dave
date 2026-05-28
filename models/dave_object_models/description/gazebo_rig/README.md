# gazebo_rig

CAD target for the 3D-sonar demo. Referenced as `model://gazebo_rig`.

Companion files:

- `models/dave_worlds/worlds/gazebo_rig_demo.world` — rig at origin.
- `gazebo/dave_gz_multibeam_sonar/sonar_3d_demo/launch/gazebo_rig_demo.launch.py`
  — spawns the 3D sonar at `x=-3, yaw=0` (bore = +X).

## Running

```bash
colcon build --packages-select dave_object_models dave_worlds sonar_3d_demo
source install/setup.bash

# Terminal 1
ros2 launch sonar_3d_demo gazebo_rig_demo.launch.py

# Terminal 2 (after multibeams initialize)
ros2 launch sonar_3d_demo viz.launch.py
```

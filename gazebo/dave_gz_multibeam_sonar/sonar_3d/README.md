# 3D Sonar for Gazebo
Ruuning the 3D sonar
```
ros2 launch dave_multibeam_sonar_demo 3d_sonar_demo.launch.py 
```

Wait until all the multibeams are initialized, then run the sonar aggregator node

```
ros2 run sonar_3d sonar_aggregator
```
import math
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge_args = []
    tf_nodes = []
    
    num_sensors = 67
    elevation_step_deg = 0.60
    
    # Map points for 67 multibeam sonars and create their TFs
    for i in range(num_sensors):
        # 1. Bridge the PointCloud
        bridge_args.append(f"/sensor/sonar_3d/beam_{i}/point_cloud_{i}@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked")
        
        # 2. Add static transform publisher for each sensor's pose
        pitch_deg = (i - (num_sensors - 1) / 2.0) * elevation_step_deg
        pitch_rad = math.radians(pitch_deg)
        
        tf_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"tf_sonar_{i}",
            arguments=[
                "0", "0", "0", "0", f"{pitch_rad:.6f}", "0", # X, Y, Z, Roll, Pitch, Yaw
                "3d_sonar/base_link",
                f"3d_sonar/base_link/sonar_3d_{i}"
            ],
            output="screen"
        )
        tf_nodes.append(tf_node)
        
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_args,
        output="screen",
    )

    tf_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            "3d_sonar/base_link",
        ],
    )

    return LaunchDescription([bridge, tf_base] + tf_nodes)

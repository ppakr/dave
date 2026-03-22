import os
import math

# Parameters based on real sonar specs
# Horizontal FOV: 90° (±45°)
# Vertical FOV: 40° (±20°)
# Beam separation: 0.35° (horizontal) / 0.60° (vertical)
# Angular resolution: 0.85° (horizontal) / 1.60° (vertical)

field_of_view_vertical_deg = 40.0
elevation_step_deg = 0.60
num_sensors = int(field_of_view_vertical_deg / elevation_step_deg) + 1 # 67 sensors

horizontal_fov_deg = 90.0
horizontal_min_angle_rad = math.radians(-45.0)
horizontal_max_angle_rad = math.radians(45.0)
horizontal_beam_separation_deg = 0.35
num_beams = int(horizontal_fov_deg / horizontal_beam_separation_deg) + 1 # 258 beams

vertical_angular_resolution_deg = 1.6 # Used for verticalFOV

out_dir = "/home/aki/auv_ws/src/dave/models/dave_sensor_models/3d_sonar"
os.makedirs(out_dir, exist_ok=True)

model_sdf_path = os.path.join(out_dir, "model.sdf")
model_config_path = os.path.join(out_dir, "model.config")

# Write model.config
config_content = f"""<?xml version="1.0"?>
<model>
  <name>3D Sonar</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Generated</name>
    <email>generated@example.com</email>
  </author>
  <description>
    {num_sensors} multibeam sonars stacked with {elevation_step_deg} degree elevation difference.
    Horizontal FOV: 90 deg, Vertical FOV: 40 deg.
  </description>
</model>
"""
with open(model_config_path, "w") as f:
    f.write(config_content)

# Start generating SDF
sdf_content = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="3d_sonar">
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <iyy>0.01</iyy>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.5</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.5</length>
          </cylinder>
        </geometry>
      </collision>
"""

# Generate sensors
for i in range(num_sensors):
    # center around 0
    pitch_deg = (i - (num_sensors - 1) / 2.0) * elevation_step_deg
    pitch_rad = math.radians(pitch_deg)
    
    sensor_xml = f"""
      <sensor name="sonar_3d_{i}" type="custom" gz:type="sonar_3d">
        <pose>0 0 0 0 {pitch_rad:.6f} 0</pose>
        <always_on>true</always_on>
        <update_rate>30.0</update_rate>
        <topic>/sensor/sonar_3d/{i}</topic>
        <visualize>true</visualize>
        <gz:sonar_3d>
        <ray degrees="false">
          <scan>
            <horizontal>
              <beams>{num_beams}</beams>
              <min_angle>{horizontal_min_angle_rad:.6f}</min_angle>
              <max_angle>{horizontal_max_angle_rad:.6f}</max_angle>
            </horizontal>
            <vertical>
              <rays>1</rays>
              <min_angle>-0.00174532925</min_angle>
              <max_angle>0.00174532925</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>10.0</max>
          </range>
        </ray>
          <spec>
            <verticalFOV>{vertical_angular_resolution_deg}</verticalFOV>
            <sonarFreq>900e3</sonarFreq>
            <bandwidth>29.9e3</bandwidth>
            <soundSpeed>1500</soundSpeed>
            <sourceLevel>220</sourceLevel>
            <maxDistance>10</maxDistance>
            <raySkips>1</raySkips>
            <sensorGain>0.02</sensorGain>
            <blazingSonarImage>true</blazingSonarImage>
            <writeLog>false</writeLog>
            <debugFlag>false</debugFlag>
            <writeFrameInterval>5</writeFrameInterval>
            <pointCloudTopicName>point_cloud_{i}</pointCloudTopicName>
            <sonarImageRawTopicName>sonar_image_raw_{i}</sonarImageRawTopicName>
            <sonarImageTopicName>sonar_image_{i}</sonarImageTopicName>
            <frameName>sonar_3d_link_{i}</frameName>
          </spec>
        </gz:sonar_3d>
      </sensor>"""
    sdf_content += sensor_xml

sdf_content += """
    </link>
  </model>
</sdf>
"""

with open(model_sdf_path, "w") as f:
    f.write(sdf_content)

print(f"Generated {num_sensors} sonar sensors in {model_sdf_path}")


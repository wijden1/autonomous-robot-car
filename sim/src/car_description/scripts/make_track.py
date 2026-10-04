"""Generate an oval track world (worlds/track.sdf) with a yellow center line."""
import math
import os

STRAIGHT = 3.0       # length of the straight parts (m)
RADIUS = 1.0         # radius of the curves (m)
LINE_WIDTH = 0.04    # width of the yellow line (m)
ARC_SEGMENTS = 24    # pieces per curve (more = smoother)

half = STRAIGHT / 2
segments = [(0.0, -RADIUS, 0.0, STRAIGHT), (0.0, RADIUS, math.pi, STRAIGHT)]
for cx, start in ((half, -math.pi / 2), (-half, math.pi / 2)):
    for i in range(ARC_SEGMENTS):
        a0 = start + math.pi * i / ARC_SEGMENTS
        a1 = start + math.pi * (i + 1) / ARC_SEGMENTS
        x0, y0 = cx + RADIUS * math.cos(a0), RADIUS * math.sin(a0)
        x1, y1 = cx + RADIUS * math.cos(a1), RADIUS * math.sin(a1)
        length = math.hypot(x1 - x0, y1 - y0) + 0.01
        segments.append(((x0 + x1) / 2, (y0 + y1) / 2, math.atan2(y1 - y0, x1 - x0), length))

line_visuals = "\n".join(
    f'        <visual name="seg_{i}"><pose>{x:.4f} {y:.4f} 0.001 0 0 {yaw:.4f}</pose>'
    f'<geometry><box><size>{length:.4f} {LINE_WIDTH} 0.002</size></box></geometry>'
    f'<material><ambient>1 0.8 0 1</ambient><diffuse>1 0.8 0 1</diffuse></material></visual>'
    for i, (x, y, yaw, length) in enumerate(segments))


def obstacle(name, x, y):
    return f"""
    <model name="{name}">
      <pose>{x} {y} 0.1 0 0 0</pose>
      <link name="link">
        <inertial><mass>1.0</mass>
          <inertia><ixx>0.007</ixx><iyy>0.007</iyy><izz>0.007</izz></inertia>
        </inertial>
        <collision name="c"><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.2 0.2 0.2</size></box></geometry>
          <material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>"""


world = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="track">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>12 8</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>12 8</size></plane></geometry>
          <material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="lane_line">
      <static>true</static>
      <link name="link">
{line_visuals}
      </link>
    </model>
{obstacle("obstacle_1", 0.0, 2.0)}
{obstacle("obstacle_2", 1.0, 2.0)}
  </world>
</sdf>
"""

out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "worlds")
os.makedirs(out_dir, exist_ok=True)
with open(os.path.join(out_dir, "track.sdf"), "w") as f:
    f.write(world)
print(f"Wrote track with {len(segments)} line segments to worlds/track.sdf")
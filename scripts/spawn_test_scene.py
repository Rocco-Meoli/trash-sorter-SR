#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

SDF_BOX = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>{static}</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

def spawn_box(name, x, y, z, sx, sy, sz, r, g, b, static=False, mass=0.2):
    sdf = SDF_BOX.format(
        name=name,
        static=str(static).lower(),
        x=x, y=y, z=z,
        sx=sx, sy=sy, sz=sz,
        r=r, g=g, b=b,
        mass=mass
    )
    pose = Pose()
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    return spawn(model_name=name, model_xml=sdf, robot_namespace="", initial_pose=pose, reference_frame="world")

def safe_delete(name):
    try:
        rospy.wait_for_service("/gazebo/delete_model", timeout=1.0)
        delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete(name)
    except Exception:
        pass

def main():
    rospy.init_node("spawn_test_scene", anonymous=True)

    # Pulizia
    for n in ["ts_table", "ts_cube", "ts_bin"]:
        safe_delete(n)

    # Tavolo: top a z=0.75 (quindi altezza 0.75)
    # (base center at z=0.375)
    spawn_box(
        name="ts_table",
        x=1.00, y=0.20, z=0.375,
        sx=0.60, sy=0.90, sz=0.75,
        r=0.7, g=0.7, b=0.7,
        static=True
    )

    # Cubetto sul tavolo: top table z=0.75, cubo size 0.04 -> center z=0.75+0.02=0.77
    spawn_box(
        name="ts_cube",
        x=1.00, y=0.20, z=0.77,
        sx=0.04, sy=0.04, sz=0.04,
        r=0.1, g=0.6, b=0.9,
        static=False,
        mass=0.05
    )

    # “Bin” come box statico a destra
    spawn_box(
        name="ts_bin",
        x=0.70, y=-0.70, z=0.40,
        sx=0.30, sy=0.30, sz=0.80,
        r=0.2, g=0.8, b=0.2,
        static=True
    )

    rospy.loginfo("Spawned: table + cube + bin.")
    rospy.loginfo("Cube pose (world): x=0.55 y=0.00 z=0.77")

if __name__ == "__main__":
    main()

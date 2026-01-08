#!/usr/bin/env python3
import sys
import math
import rospy
import actionlib
import moveit_commander
import cv2
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from gazebo_msgs.srv import GetModelState
from tf.transformations import quaternion_from_euler

from franka_gripper.msg import MoveAction, MoveGoal
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ===============================
# PARAMETRI MONDO
# ===============================
OBJ_DIAMETERS = {
    "orange":   0.055,  # ora: TRAPANO (ma manteniamo nome per compatibilità)
    "plastic_bottle": 0.050,  # Coke
    "carton_pack":    0.050,  # Latte
}

# Offset di discesa (quanto sopra il centro modello andare per chiudere)
# Per il trapano sdraiato conviene stare un filo più alto
OBJ_Z_GRAB_OFFSET = {
    "orange":   0.12,   # trapano
    "plastic_bottle": 0.06,   # lattina
    "carton_pack":    0.03,   # cartone
}

# Offset centro-presa SOLO per arancia (glass_bottle) in metri
# Y negativo = più a sinistra (nel tuo mondo)
ORANGE_PICK_DX = 0.00    # es. 0.01 = 1 cm in avanti
ORANGE_PICK_DY = -0.015   # es. -0.02 = 2 cm verso sinistra


GAZEBO_ATTACH_LINK = "panda_link7"
MID_JOINTS_SAFE = [0.0, -0.7, 0.0, -2.1, 0.0, 1.6, 0.8]
CAM_TOPIC = "/overhead_camera/image_raw"

DEFAULT_PRE_DZ  = 0.18
DEFAULT_LIFT_DZ = 0.22

# ===============================
# SCENE HELPER
# ===============================
class UltraSceneHelper:
    def __init__(self):
        self.scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        rospy.wait_for_service('/get_planning_scene')
        self.get_scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)

    def add_table_to_moveit(self, scene):
        p = PoseStamped()
        p.header.frame_id = "world"
        p.pose.position.x, p.pose.position.y, p.pose.position.z = 0.60, 0, 0.20
        scene.add_box("work_table", p, size=(0.5, 0.8, 0.4))

    def allow_all_collisions(self, obj_name, allow=True):
        req = GetPlanningSceneRequest()
        req.components.components = req.components.ALLOWED_COLLISION_MATRIX
        res = self.get_scene_srv(req)
        acm = res.scene.allowed_collision_matrix

        if obj_name not in acm.entry_names:
            acm.entry_names.append(obj_name)
            for entry in acm.entry_values:
                entry.enabled.append(False)
            new_entry = AllowedCollisionEntry()
            new_entry.enabled = [False] * len(acm.entry_names)
            acm.entry_values.append(new_entry)

        idx_obj = acm.entry_names.index(obj_name)
        for i in range(len(acm.entry_names)):
            acm.entry_values[idx_obj].enabled[i] = allow
            acm.entry_values[i].enabled[idx_obj] = allow

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.allowed_collision_matrix = acm
        self.scene_pub.publish(scene_msg)
        rospy.sleep(0.2)

# ===============================
# UTILS
# ===============================
def get_model_xyz(name):
    rospy.wait_for_service("/gazebo/get_model_state")
    srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    r = srv(name, "world")
    return r.pose.position.x, r.pose.position.y, r.pose.position.z

def gripper_control(c, width, speed=0.1):
    c.send_goal(MoveGoal(width=width, speed=speed))
    c.wait_for_result(rospy.Duration(2.0))

def gazebo_attach_srv(obj, attach=True):
    name = "/link_attacher_node/attach" if attach else "/link_attacher_node/detach"
    rospy.wait_for_service(name)
    srv = rospy.ServiceProxy(name, Attach)
    srv(AttachRequest(model_name_1="panda",
                      link_name_1=GAZEBO_ATTACH_LINK,
                      model_name_2=obj,
                      link_name_2="link"))

def make_pose(x, y, z, q: Quaternion):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation = q
    return p

def execute_cartesian(arm, waypoints, eef_step=0.005, avoid_collisions=False):
    # tua MoveIt: (list, double, bool)
    plan, fraction = arm.compute_cartesian_path(waypoints, float(eef_step), bool(avoid_collisions))
    if fraction < 0.95:
        rospy.logwarn(f"Cartesian fraction bassa: {fraction:.2f}")
    arm.execute(plan, wait=True)

# ===============================
# PICK AND PLACE (UNIFICATO)
# ===============================
def pick_and_place(obj, binm, arm, scene, gripper, scene_helper, eef):
    rospy.loginfo(f"\n--- AVVIO TASK: {obj} ---")

    try:
        arm.stop()
        arm.set_start_state_to_current_state()

        # velocità conservative
        arm.set_max_velocity_scaling_factor(0.03)
        arm.set_max_acceleration_scaling_factor(0.02)

        ox, oy, oz = get_model_xyz(obj)
        bx, by, bz = get_model_xyz(binm)

        # Orientazione “standard” come il tuo codice per lattina/cartone
        q_std = Quaternion(*quaternion_from_euler(0, math.pi, 1.57))

        # 1) APPROCCIO ALTO
        # >>> SOLO ARANCIA: applico offset su X/Y per centrare tra le dita
        if obj == "orange":
            px = ox + ORANGE_PICK_DX
            py = oy + ORANGE_PICK_DY
            rospy.logwarn(f"[ARANCIA] offset pick dx={ORANGE_PICK_DX:.3f} dy={ORANGE_PICK_DY:.3f} "
                          f"target=({px:.3f},{py:.3f}) original=({ox:.3f},{oy:.3f})")
        else:
            px, py = ox, oy

        pre_pick = make_pose(px, py, oz + DEFAULT_PRE_DZ, q_std)
        arm.set_pose_target(pre_pick)
        arm.go(wait=True)

        gripper_control(gripper, 0.08)

        # 2) DISCESA CONTROLLATA (come prima)
        scene_helper.allow_all_collisions(obj, True)
        arm.set_start_state_to_current_state()

        z_grab = oz + OBJ_Z_GRAB_OFFSET.get(obj, 0.02)
        touch_pose = make_pose(px, py, z_grab, q_std)


        # per il trapano facciamo passi un pelo più fini
        eef_step = 0.004 if obj == "glass_bottle" else 0.005
        execute_cartesian(arm, [touch_pose], eef_step=eef_step, avoid_collisions=False)
        rospy.sleep(0.1)

        # 3) PRESA
        grasp_w = OBJ_DIAMETERS.get(obj, 0.05) - 0.005
        gripper_control(gripper, grasp_w, speed=0.02)
        rospy.sleep(0.2)

        gazebo_attach_srv(obj, True)
        rospy.sleep(0.1)

        # 4) SOLLEVAMENTO
        arm.set_start_state_to_current_state()
        lift_pose = make_pose(ox, oy, oz + DEFAULT_LIFT_DZ, q_std)
        execute_cartesian(arm, [lift_pose], eef_step=0.01, avoid_collisions=False)

        # 5) TRASPORTO
        arm.set_max_velocity_scaling_factor(0.12)
        arm.set_max_acceleration_scaling_factor(0.08)
        arm.set_start_state_to_current_state()

        arm.set_joint_value_target(MID_JOINTS_SAFE)
        arm.go(wait=True)

        move_to_bin = make_pose(bx, by, bz + 0.40, q_std)
        arm.set_pose_target(move_to_bin)
        arm.go(wait=True)

        # 6) RILASCIO
        try:
            scene.remove_attached_object(eef, obj)
        except Exception:
            pass

        gazebo_attach_srv(obj, False)
        rospy.sleep(0.05)
        gripper_control(gripper, 0.08)
        rospy.sleep(0.05)

        arm.set_named_target("ready")
        arm.go(wait=True)
        rospy.loginfo(f"✓ Task completato: {obj}")

    except Exception as e:
        rospy.logerr(f"ERRORE: {e}")
        arm.stop()
        try:
            gazebo_attach_srv(obj, False)
        except Exception:
            pass
        arm.set_named_target("ready")
        arm.go(wait=True)

# ===============================
# VISIONE (ordine forzato: glass_bottle primo)
# ===============================
class OverheadVision:
    def __init__(self, topic=CAM_TOPIC):
        self.bridge = CvBridge()
        self.last_img = None
        self.sub = rospy.Subscriber(topic, Image, self.cb)

    def cb(self, msg):
        self.last_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def build_tasks(self):
        rospy.sleep(2.0)
        if self.last_img is None:
            return []

        hsv = cv2.cvtColor(self.last_img, cv2.COLOR_BGR2HSV)

        # Range più permissivi (trapano con parti nere/grigie e arancione opaco)
        ranges = {
            "orange": ((5, 60, 60), (35, 255, 255)),
            "red":    ((0, 80, 80), (10, 255, 255)),
            "black":  ((0, 0, 0), (180, 255, 60)),
        }

        kernel = np.ones((5, 5), np.uint8)

        # det[colore] = (cx, area)
        det = {}

        for name, (lo, hi) in ranges.items():
            mask = cv2.inRange(hsv, np.array(lo), np.array(hi))

            # rosso: doppia fascia hue (0-10) U (160-180)
            if name == "red":
                mask2 = cv2.inRange(hsv, np.array((160, 80, 80)), np.array((180, 255, 255)))
                mask = cv2.bitwise_or(mask, mask2)

            # Unisce pezzi e ripulisce rumore
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)

            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue

            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area < 150:  # soglia più robusta di 50
                continue

            M = cv2.moments(c)
            if M["m00"] <= 0:
                continue

            cx = int(M["m10"] / M["m00"])
            det[name] = (cx, area)

        mapping = {"orange": "orange", "red": "plastic_bottle", "black": "carton_pack"}
        bins    = {"orange": "bin_brown",     "red": "bin_yellow",     "black": "bin_blue"}

        # colori trovati (ordinati per posizione orizzontale nell'immagine)
        found_colors = list(det.keys())
        sorted_colors = sorted(found_colors, key=lambda k: det[k][0], reverse=True)

        tasks = [(mapping[c], bins[c]) for c in sorted_colors if c in mapping]

        # Forza ordine: glass_bottle -> lattina -> cartone (come prima)
        ordered = []
        for obj_name in ["orange", "plastic_bottle", "carton_pack"]:
            for t in tasks:
                if t[0] == obj_name:
                    ordered.append(t)

        rospy.loginfo(f"Task rilevati (ordine forzato): {ordered}")
        return ordered


def main():
    rospy.init_node("panda_trash_sort")
    moveit_commander.roscpp_initialize(sys.argv)

    arm = moveit_commander.MoveGroupCommander("panda_arm")
    scene = moveit_commander.PlanningSceneInterface()
    scene_helper = UltraSceneHelper()
    eef = arm.get_end_effector_link()

    gripper = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
    gripper.wait_for_server()

    # Setup scene
    try:
        scene.remove_attached_object(eef)
    except Exception:
        pass

    for o in scene.get_known_object_names():
        scene.remove_world_object(o)
    rospy.sleep(1.0)

    scene_helper.add_table_to_moveit(scene)

    arm.set_named_target("ready")
    arm.go(wait=True)

    vision = OverheadVision()
    tasks = vision.build_tasks()

    if not tasks:
        # fallback: glass_bottle primo
        tasks = [("glass_bottle", "bin_green"),
                 ("plastic_bottle", "bin_yellow"),
                 ("carton_pack", "bin_blue")]

    for obj, binm in tasks:
        pick_and_place(obj, binm, arm, scene, gripper, scene_helper, eef)

if __name__ == "__main__":
    main()

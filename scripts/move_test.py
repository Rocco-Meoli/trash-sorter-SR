#!/usr/bin/env python3
import sys
import math
import rospy
import actionlib
import moveit_commander
import cv2
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler

from franka_gripper.msg import MoveAction, MoveGoal
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ===============================
# PARAMETRI MONDO (SDF Sync)
# ===============================
OBJ_BB = {
    "glass_bottle":   (0.06, 0.06, 0.18),
    "plastic_bottle": (0.056, 0.056, 0.18),
    "carton_pack":    (0.05, 0.05, 0.16),
}

GRIPPER_TOUCH_LINKS = ["panda_leftfinger", "panda_rightfinger", "panda_hand"]
GAZEBO_ATTACH_LINK = "panda_link7"
MID_JOINTS_SAFE = [0.0, -0.7, 0.0, -2.1, 0.0, 1.6, 0.8]
CAM_TOPIC = "/overhead_camera/image_raw"

# ===============================
# SCENE & ACM HELPER
# ===============================
class UltraSceneHelper:
    def __init__(self):
        self.scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        rospy.wait_for_service('/get_planning_scene')
        self.get_scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)

    def allow_all_collisions(self, obj_name, allow=True):
        """Disabilita ogni controllo collisione per l'oggetto specificato"""
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
        rospy.sleep(0.4)

# ===============================
# UTILS
# ===============================
def get_model_xyz(name):
    rospy.wait_for_service("/gazebo/get_model_state")
    srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    r = srv(name, "world")
    return r.pose.position.x, r.pose.position.y, r.pose.position.z

def make_pose(x, y, z):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    q = quaternion_from_euler(0, math.pi, 0)
    p.orientation = Quaternion(*q)
    return p

def gripper_control(c, width):
    c.send_goal(MoveGoal(width=width, speed=0.1))
    c.wait_for_result(rospy.Duration(2.0))

def gazebo_attach_srv(obj, attach=True):
    name = "/link_attacher_node/attach" if attach else "/link_attacher_node/detach"
    rospy.wait_for_service(name)
    srv = rospy.ServiceProxy(name, Attach)
    srv(AttachRequest(model_name_1="panda", link_name_1=GAZEBO_ATTACH_LINK, model_name_2=obj, link_name_2="link"))

# ===============================
# VISION
# ===============================
class OverheadVision:
    def __init__(self, topic=CAM_TOPIC):
        self.bridge = CvBridge()
        self.last_img = None
        self.sub = rospy.Subscriber(topic, Image, self.cb)

    def cb(self, msg):
        self.last_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def build_tasks(self):
        rospy.sleep(1.0)
        if self.last_img is None: return []
        hsv = cv2.cvtColor(self.last_img, cv2.COLOR_BGR2HSV)
        ranges = {"green": ((35,80,80),(85,255,255)), "yellow": ((18,80,80),(35,255,255)), "blue": ((90,80,80),(130,255,255))}
        det = {}
        for name, (lo, hi) in ranges.items():
            mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                M = cv2.moments(c)
                det[name] = int(M["m10"]/M["m00"]) if M["m00"]>0 else None
        sorted_colors = sorted([k for k,v in det.items() if v is not None], key=lambda x: det[x])
        mapping = {"green": "glass_bottle", "yellow": "plastic_bottle", "blue": "carton_pack"}
        bins = {"green": "bin_green", "yellow": "bin_yellow", "blue": "bin_blue"}
        return [(mapping[c], bins[c]) for c in sorted_colors]

# ===============================
# PICK AND PLACE
# ===============================
def pick_and_place(obj, binm, arm, scene, gripper, scene_helper, eef):
    rospy.loginfo(f"\n--- AVVIO TASK: {obj} -> {binm} ---")
    in_gazebo = False
    in_moveit = False
    
    try:
        ox, oy, oz = get_model_xyz(obj)
        bx, by, bz = get_model_xyz(binm)

        # 1. APPROCCIO ALTO
        arm.set_max_velocity_scaling_factor(0.15)
        arm.set_start_state_to_current_state()
        pre_pick = make_pose(ox, oy, oz + 0.25)
        arm.set_pose_target(pre_pick)
        arm.go(wait=True)

        # 2. DISATTIVA COLLISIONI
        # Diciamo a MoveIt di ignorare l'oggetto completamente durante la presa
        scene_helper.allow_all_collisions(obj, True)

        # 3. DISCESA LINEARE (Ignora tutto)
        arm.set_max_velocity_scaling_factor(0.04)
        touch_pose = make_pose(ox, oy, oz + 0.08)
        (plan, fraction) = arm.compute_cartesian_path([touch_pose], 0.01, False)
        arm.execute(plan, wait=True)
        rospy.sleep(0.5)

        # 4. PRESA FISICA
        gripper_control(gripper, 0.038) # Larghezza sicura
        rospy.sleep(0.6)
        gazebo_attach_srv(obj, True)
        in_gazebo = True

        # 5. SOLLEVAMENTO FORZATO (Anti START_STATE_IN_COLLISION)
        # Saliamo di molto ignorando le collisioni
        lift_pose = make_pose(ox, oy, oz + 0.35)
        (plan, fraction) = arm.compute_cartesian_path([lift_pose], 0.01, False)
        if fraction > 0.3:
            arm.execute(plan, wait=True)
        else:
            rospy.logwarn("Cartesiano difficile, forzo PTP per salire...")
            arm.set_pose_target(lift_pose)
            arm.go(wait=True)
        rospy.sleep(0.5)

        # 6. ATTACH IN MOVEIT (Solo ora che siamo liberi in aria)
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose = arm.get_current_pose().pose
        scene.add_box(obj, ps, size=OBJ_BB[obj])
        scene.attach_box(eef, obj, touch_links=GRIPPER_TOUCH_LINKS)
        in_moveit = True
        rospy.sleep(0.5)

        # 7. TRASPORTO (Reset Stato)
        arm.set_max_velocity_scaling_factor(0.2)
        arm.set_start_state_to_current_state()
        
        # Passaggio Joint per evitare rotazioni strane
        arm.set_joint_value_target(MID_JOINTS_SAFE)
        arm.go(wait=True)

        # Vai sopra il contenitore
        move_to_bin = make_pose(bx, by, bz + 0.45)
        arm.set_pose_target(move_to_bin)
        arm.go(wait=True)

        # Discesa nel contenitore
        drop_pose = make_pose(bx, by, bz + 0.28)
        arm.set_pose_target(drop_pose)
        arm.go(wait=True)

        # 8. RILASCIO
        gripper_control(gripper, 0.08)
        gazebo_attach_srv(obj, False)
        if in_moveit:
            scene.remove_attached_object(eef, obj)
            scene.remove_world_object(obj)
            in_moveit = False
        in_gazebo = False
        scene_helper.allow_all_collisions(obj, False)
        rospy.loginfo(f"✓ Task completato: {obj}")

        # 9. RESET POSIZIONE
        arm.set_named_target("ready")
        arm.go(wait=True)

    except Exception as e:
        rospy.logerr(f"Task interrotto: {e}")
        if in_gazebo: gazebo_attach_srv(obj, False)
        scene.remove_attached_object(eef)
        scene.remove_world_object()
        arm.set_named_target("ready")
        arm.go(wait=True)

# ===============================
# MAIN
# ===============================
def main():
    rospy.init_node("panda_ultra_safe_pick")
    moveit_commander.roscpp_initialize(sys.argv)
    
    arm = moveit_commander.MoveGroupCommander("panda_arm")
    scene = moveit_commander.PlanningSceneInterface()
    scene_helper = UltraSceneHelper()
    eef = arm.get_end_effector_link()
    
    gripper = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
    rospy.loginfo("In attesa del gripper...")
    gripper.wait_for_server()

    # Pulizia iniziale Planning Scene
    scene.remove_attached_object(eef)
    scene.remove_world_object()
    rospy.sleep(1.0)
    
    arm.set_named_target("ready")
    arm.go(wait=True)

    vision = OverheadVision()
    tasks = vision.build_tasks()
    if not tasks:
        tasks = [("glass_bottle", "bin_green"), ("plastic_bottle", "bin_yellow"), ("carton_pack", "bin_blue")]

    for obj, binm in tasks:
        pick_and_place(obj, binm, arm, scene, gripper, scene_helper, eef)

    rospy.loginfo("--- TUTTI I TASK COMPLETATI ---")

if __name__ == "__main__":
    main()
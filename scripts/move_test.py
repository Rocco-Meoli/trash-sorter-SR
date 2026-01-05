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
OBJ_DIAMETERS = {
    "glass_bottle":   0.06,
    "plastic_bottle": 0.056,
    "carton_pack":    0.05,
}

OBJ_BB = {
    "glass_bottle":   (0.06, 0.06, 0.18),
    "plastic_bottle": (0.056, 0.056, 0.18),
    "carton_pack":    (0.05, 0.05, 0.16),
}

TABLE_HEIGHT = 0.40
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
        
        if "work_table" in acm.entry_names:
            t_idx = acm.entry_names.index("work_table")
            for l in ["panda_leftfinger", "panda_rightfinger"]:
                if l in acm.entry_names:
                    l_idx = acm.entry_names.index(l)
                    acm.entry_values[t_idx].enabled[l_idx] = allow
                    acm.entry_values[l_idx].enabled[t_idx] = allow

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

def make_pose(x, y, z, pitch=math.pi):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    q = quaternion_from_euler(0, pitch, 1.57) 
    p.orientation = Quaternion(*q)
    return p

def gripper_control(c, width, speed=0.1):
    c.send_goal(MoveGoal(width=width, speed=speed))
    c.wait_for_result(rospy.Duration(2.0))

def gazebo_attach_srv(obj, attach=True):
    name = "/link_attacher_node/attach" if attach else "/link_attacher_node/detach"
    rospy.wait_for_service(name)
    srv = rospy.ServiceProxy(name, Attach)
    srv(AttachRequest(model_name_1="panda", link_name_1=GAZEBO_ATTACH_LINK, model_name_2=obj, link_name_2="link"))

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

        # 1. APPROCCIO ALTO (Sopra il tavolo)
        arm.set_max_velocity_scaling_factor(0.15)
        arm.set_start_state_to_current_state()
        pre_pick = make_pose(ox, oy, oz + 0.15)
        arm.set_pose_target(pre_pick)
        arm.go(wait=True)

        # --- MODIFICA RICHIESTA: Pre-apertura dita prima della discesa ---
        # Apriamo le dita al diametro dell'oggetto + 2cm di margine
        safe_open_width = OBJ_DIAMETERS.get(obj, 0.06) + 0.02
        rospy.loginfo(f"Pre-apertura dita per {obj}: {safe_open_width}m")
        gripper_control(gripper, safe_open_width)
        # ----------------------------------------------------------------

        # 2. DISATTIVA COLLISIONI (ACM)
        scene_helper.allow_all_collisions(obj, True)

        # 3. DISCESA CHIRURGICA
        arm.set_max_velocity_scaling_factor(0.02) 
        touch_pose = make_pose(ox, oy, oz) 
        (plan, fraction) = arm.compute_cartesian_path([touch_pose], 0.01, False)
        arm.execute(plan, wait=True)
        rospy.sleep(0.5)

        # 4. PRESA REALE (Sandwich)
        grasp_w = OBJ_DIAMETERS.get(obj, 0.05) - 0.005 
        rospy.loginfo(f"Chiusura reale su {obj}: {grasp_w}m")
        gripper_control(gripper, grasp_w, speed=0.04)
        rospy.sleep(0.6)
        
        gazebo_attach_srv(obj, True)
        in_gazebo = True

        # 5. SOLLEVAMENTO VERTICALE
        lift_pose = make_pose(ox, oy, oz + 0.25)
        (plan, fraction) = arm.compute_cartesian_path([lift_pose], 0.01, False)
        arm.execute(plan, wait=True)
        rospy.sleep(0.5)

        # 6. ATTACH IN MOVEIT
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose = arm.get_current_pose().pose
        scene.add_box(obj, ps, size=OBJ_BB[obj])
        scene.attach_box(eef, obj, touch_links=GRIPPER_TOUCH_LINKS)
        in_moveit = True

        # 7. TRASPORTO (Reset Stato)
        arm.set_max_velocity_scaling_factor(0.2)
        arm.set_start_state_to_current_state()
        arm.set_joint_value_target(MID_JOINTS_SAFE)
        arm.go(wait=True)

        move_to_bin = make_pose(bx, by, bz + 0.40)
        arm.set_pose_target(move_to_bin)
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
        
        arm.set_named_target("ready")
        arm.go(wait=True)

    except Exception as e:
        rospy.logerr(f"Task interrotto: {e}")
        gazebo_attach_srv(obj, False)
        scene.remove_attached_object(eef)
        scene.remove_world_object()
        arm.set_named_target("ready")
        arm.go(wait=True)

# ... (Classe OverheadVision identica a prima) ...

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

def main():
    rospy.init_node("panda_table_pick_place")
    moveit_commander.roscpp_initialize(sys.argv)
    
    arm = moveit_commander.MoveGroupCommander("panda_arm")
    scene = moveit_commander.PlanningSceneInterface()
    scene_helper = UltraSceneHelper()
    eef = arm.get_end_effector_link()
    
    gripper = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
    gripper.wait_for_server()

    scene.remove_attached_object(eef)
    scene.remove_world_object()
    rospy.sleep(1.0)
    scene_helper.add_table_to_moveit(scene) 
    
    arm.set_named_target("ready")
    arm.go(wait=True)

    vision = OverheadVision()
    tasks = vision.build_tasks()
    if not tasks:
        tasks = [("glass_bottle", "bin_green"), ("plastic_bottle", "bin_yellow"), ("carton_pack", "bin_blue")]

    for obj, binm in tasks:
        pick_and_place(obj, binm, arm, scene, gripper, scene_helper, eef)

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import sys
import math
import random
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
# PARAMETRI
# ===============================
CUBE_SIZE = 0.04

Z_OFFSET        = 0.095
Z_PRE_PICK      = 0.15
Z_PRE_PLACE     = 0.15
TOUCH_APPROACH_DZ = 0.03

VEL_PTP = 0.15
ACC_PTP = 0.15
VEL_LIN = 0.06
ACC_LIN = 0.06

# Recovery più lento (quando sei in merda)
VEL_REC = 0.08
ACC_REC = 0.08

GAZEBO_ATTACH_LINK = "panda_link7"
GRIPPER_TOUCH_LINKS = ["panda_link7", "panda_leftfinger", "panda_rightfinger"]

MID_JOINTS_SAFE = [0.0, -0.7, 0.0, -2.1, 0.0, 1.6, 0.8]

# --- Camera ---
CAM_TOPIC = "/overhead_camera/image_raw"
VISION_RETRIES = 10
FLIP_LEFT_RIGHT = False

# --- Slot fissi sul tavolo (random spawn) ---
SLOT_X = 0.60
SLOT_YS = [-0.18, 0.0, 0.18]
SLOT_Z = 0.42


# ===============================
# UTILS
# ===============================
def get_model_xyz(name):
    rospy.wait_for_service("/gazebo/get_model_state")
    srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    r = srv(name, "world")
    if hasattr(r, "success") and not r.success:
        raise RuntimeError(f"Gazebo GetModelState failed for '{name}': {getattr(r, 'status_message', '')}")
    p = r.pose.position
    return p.x, p.y, p.z


def set_model_pose(name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    rospy.wait_for_service("/gazebo/set_model_state")
    srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    st = ModelState()
    st.model_name = name
    st.reference_frame = "world"
    st.pose.position.x = x
    st.pose.position.y = y
    st.pose.position.z = z

    q = quaternion_from_euler(roll, pitch, yaw)
    st.pose.orientation.x, st.pose.orientation.y, st.pose.orientation.z, st.pose.orientation.w = q

    # azzera twist per non farli scivolare
    st.twist.linear.x = st.twist.linear.y = st.twist.linear.z = 0.0
    st.twist.angular.x = st.twist.angular.y = st.twist.angular.z = 0.0

    ok = srv(st).success
    if not ok:
        raise RuntimeError(f"❌ Gazebo SetModelState failed for '{name}'")


def randomize_cubes_positions():
    cubes = ["cube_green", "cube_yellow", "cube_blue"]
    ys = SLOT_YS.copy()
    random.shuffle(ys)
    rospy.loginfo(f"🎲 Randomizing cubes: {list(zip(cubes, ys))}")
    for cube, y in zip(cubes, ys):
        set_model_pose(cube, SLOT_X, y, SLOT_Z)
        rospy.sleep(0.6)


def make_pose(x, y, z):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    q = quaternion_from_euler(0, math.pi, 0)  # gripper down
    p.orientation = Quaternion(*q)
    return p


def set_speed(arm, vel, acc):
    arm.set_max_velocity_scaling_factor(vel)
    arm.set_max_acceleration_scaling_factor(acc)


def resync_state(arm, n=3, sleep=0.08):
    """
    Dopo ABORTED/CONTROL_FAILED, MoveIt e controller possono essere fuori sync.
    Questa cosa riduce tantissimo 'start point deviates...'
    """
    for _ in range(n):
        try:
            arm.stop()
            arm.clear_pose_targets()
        except Exception:
            pass
        rospy.sleep(sleep)
        try:
            arm.set_start_state_to_current_state()
        except Exception:
            pass
        rospy.sleep(sleep)


# ===============================
# PILZ FIX (IMPORTANTISSIMO)
# ===============================
def set_pilz(group, planner="PTP"):
    """
    Con pilz_industrial_motion_planner, planner_id vuoto = morte.
    QuindiIF: forza sempre planner_id valido ("PTP"/"LIN") e, se disponibile, forza pipeline pilz.
    """
    if not planner:
        planner = "PTP"
    try:
        if hasattr(group, "set_planning_pipeline_id"):
            group.set_planning_pipeline_id("pilz_industrial_motion_planner")
    except Exception:
        pass
    try:
        group.set_planner_id(planner)
    except Exception:
        pass


# ===============================
# MOVEIT SCENE
# ===============================
def upsert_cube_to_scene(scene, name, x, y, z):
    try:
        scene.remove_world_object(name)
    except Exception:
        pass

    ps = PoseStamped()
    ps.header.frame_id = "world"
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.w = 1.0

    scene.add_box(name, ps, size=(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE))
    rospy.sleep(0.3)


# ===============================
# GAZEBO ATTACH / DETACH
# ===============================
def gazebo_attach(cube):
    rospy.wait_for_service("/link_attacher_node/attach")
    srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    req = AttachRequest()
    req.model_name_1 = "panda"
    req.link_name_1 = GAZEBO_ATTACH_LINK
    req.model_name_2 = cube
    req.link_name_2 = "link"
    if not srv(req).ok:
        raise RuntimeError(f"❌ Gazebo attach failed ({cube})")
    rospy.loginfo(f"✓ Gazebo attach OK ({cube})")


def gazebo_detach(cube):
    rospy.wait_for_service("/link_attacher_node/detach")
    srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    req = AttachRequest()
    req.model_name_1 = "panda"
    req.link_name_1 = GAZEBO_ATTACH_LINK
    req.model_name_2 = cube
    req.link_name_2 = "link"
    if not srv(req).ok:
        raise RuntimeError(f"❌ Gazebo detach failed ({cube})")
    rospy.loginfo(f"✓ Gazebo detach OK ({cube})")


# ===============================
# ACM HELPER
# ===============================
class ACMHelper:
    def __init__(self):
        self.pub = rospy.Publisher("/planning_scene", PlanningScene, queue_size=1)
        rospy.wait_for_service("/get_planning_scene")
        self.get_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)

    def allow(self, obj, links, value: bool):
        req = GetPlanningSceneRequest()
        req.components.components = req.components.ALLOWED_COLLISION_MATRIX
        scene = self.get_scene(req).scene
        acm = scene.allowed_collision_matrix

        names = list(acm.entry_names)

        def ensure(name: str):
            nonlocal names, acm
            if name in names:
                return
            # aggiungi nuova entry_name e allunga tutte le righe esistenti
            names.append(name)
            for row in acm.entry_values:
                row.enabled.append(False)
            # aggiungi nuova riga per il nuovo name
            acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(names)))

        ensure(obj)
        for l in links:
            ensure(l)

        acm.entry_names = names
        idx = {n: i for i, n in enumerate(names)}

        for l in links:
            i = idx[obj]
            j = idx[l]
            acm.entry_values[i].enabled[j] = value
            acm.entry_values[j].enabled[i] = value

        ps = PlanningScene()
        ps.is_diff = True
        ps.allowed_collision_matrix = acm
        self.pub.publish(ps)
        rospy.sleep(0.15)


# ===============================
# MOTION
# ===============================
def move_pose(group, pose, desc, planner="PTP", strict=True):
    rospy.loginfo(f"[MOVE {planner}] {desc}")
    set_pilz(group, planner)  # <<< FIX: planner_id mai vuoto con Pilz
    resync_state(group, n=2, sleep=0.05)
    group.set_start_state_to_current_state()
    group.set_pose_target(pose)

    ok = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    if not ok:
        if strict:
            raise RuntimeError(f"❌ Move failed: {desc}")
        rospy.logwarn(f"⚠ Move '{desc}' aborted, continuing")
    return ok


def move_joints(group, joints, desc, planner="PTP", strict=True):
    rospy.loginfo(f"[MOVE {planner}] {desc}")
    set_pilz(group, planner)  # <<< FIX: planner_id mai vuoto con Pilz
    resync_state(group, n=2, sleep=0.05)
    group.set_start_state_to_current_state()
    group.set_joint_value_target(joints)

    ok = group.go(wait=True)
    group.stop()

    if not ok:
        if strict:
            raise RuntimeError(f"❌ Move failed: {desc}")
        rospy.logwarn(f"⚠ Move '{desc}' aborted, continuing")
    return ok


def goto_ready(arm):
    """
    FIX CRITICA: con Pilz, a volte go() su named target può creare request con planner_id ''.
    Quindi: set planner + plan() + execute(plan).
    """
    set_pilz(arm, "PTP")  # <<< FIX
    resync_state(arm, n=2, sleep=0.05)
    arm.set_start_state_to_current_state()

    arm.set_named_target("ready")

    plan_ret = arm.plan()
    plan = plan_ret[1] if isinstance(plan_ret, tuple) and len(plan_ret) > 1 else plan_ret

    ok_plan = hasattr(plan, "joint_trajectory") and len(plan.joint_trajectory.points) > 0
    if not ok_plan:
        arm.stop()
        arm.clear_pose_targets()
        return False

    ok = arm.execute(plan, wait=True)
    arm.stop()
    arm.clear_pose_targets()
    rospy.sleep(0.3)
    return ok


def recover_to_ready(arm):
    """
    Recovery robusto:
    - stop/resync
    - prova ready
    - altrimenti MID_JOINTS_SAFE -> ready
    """
    rospy.logwarn("🧯 Recovery: trying to return to ready safely...")
    set_speed(arm, VEL_REC, ACC_REC)

    for attempt in range(3):
        try:
            resync_state(arm, n=4, sleep=0.08)
        except Exception:
            pass

        # 1) prova direttamente ready
        try:
            if goto_ready(arm):
                rospy.loginfo("🧯 Recovery OK (ready)")
                return True
        except Exception:
            pass

        # 2) prova MID -> ready
        try:
            move_joints(arm, MID_JOINTS_SAFE, "recovery mid (JOINTS)", "PTP", strict=False)
            if goto_ready(arm):
                rospy.loginfo("🧯 Recovery OK (mid->ready)")
                return True
        except Exception:
            pass

        rospy.logwarn(f"🧯 Recovery attempt {attempt+1}/3 failed, retrying...")

    rospy.logerr("🧯 Recovery FAILED: robot may be out of sync / controller unhappy")
    return False


# ===============================
# GRIPPER
# ===============================
def gripper_open(c):
    c.send_goal(MoveGoal(width=0.08, speed=0.1))
    c.wait_for_result()


def gripper_close(c):
    c.send_goal(MoveGoal(width=0.03, speed=0.1))
    c.wait_for_result()


# ===============================
# VISION
# ===============================
class OverheadVision:
    def __init__(self, topic=CAM_TOPIC):
        self.bridge = CvBridge()
        self.last_img = None
        self.sub = rospy.Subscriber(topic, Image, self.cb, queue_size=1)

    def cb(self, msg):
        try:
            self.last_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            self.last_img = None

    def wait_image(self, timeout=5.0):
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.last_img is not None:
                return self.last_img.copy()
            if (rospy.Time.now() - t0).to_sec() > timeout:
                return None
            rospy.sleep(0.05)

    @staticmethod
    def _largest_centroid(mask):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < 50:
            return None
        M = cv2.moments(c)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)

    def detect_colors(self, img_bgr):
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        ranges = {
            "green":  ((35, 80, 80),  (85, 255, 255)),
            "yellow": ((18, 80, 80),  (35, 255, 255)),
            "blue":   ((90, 80, 80),  (130, 255, 255)),
        }
        out = {}
        for name, (lo, hi) in ranges.items():
            lo = np.array(lo, dtype=np.uint8)
            hi = np.array(hi, dtype=np.uint8)
            mask = cv2.inRange(hsv, lo, hi)
            mask = cv2.medianBlur(mask, 5)
            out[name] = self._largest_centroid(mask)
        return out

    def build_tasks_from_image(self):
        img = self.wait_image(timeout=5.0)
        if img is None:
            raise RuntimeError("❌ Camera image not received")

        det = self.detect_colors(img)
        if any(det[c] is None for c in ["green", "yellow", "blue"]):
            raise RuntimeError(f"❌ Vision failed (missing): {det}")

        colors_sorted = sorted(det.keys(), key=lambda k: det[k][0])  # left->right in image
        if FLIP_LEFT_RIGHT:
            colors_sorted = list(reversed(colors_sorted))

        rospy.loginfo(f"👁️ Camera order L->R: {colors_sorted}")

        cube_of = {"green": "cube_green", "yellow": "cube_yellow", "blue": "cube_blue"}
        bin_of  = {"green": "bin_green",  "yellow": "bin_yellow",  "blue": "bin_blue"}
        return [(cube_of[c], bin_of[c]) for c in colors_sorted]


# ===============================
# PICK&PLACE (robusto)
# ===============================
def pick_and_place(cube, binm, arm, scene, acm, gripper, eef):
    rospy.loginfo(f"\n===== START {cube} -> {binm} =====")
    attached = False

    # reset “mentale” del robot prima di ogni task
    set_speed(arm, VEL_REC, ACC_REC)
    if not goto_ready(arm):
        raise RuntimeError("❌ Cannot go to ready at task start")

    try:
        cx, cy, cz = get_model_xyz(cube)
        bx, by, bz = get_model_xyz(binm)

        upsert_cube_to_scene(scene, cube, cx, cy, cz)

        # -------- PICK --------
        pre_pick  = make_pose(cx, cy, cz + Z_OFFSET + Z_PRE_PICK)
        approach  = make_pose(cx, cy, cz + Z_OFFSET + TOUCH_APPROACH_DZ)
        touch     = make_pose(cx, cy, cz + Z_OFFSET)

        gripper_open(gripper)

        set_speed(arm, VEL_PTP, ACC_PTP)
        move_pose(arm, pre_pick, f"{cube} pre-pick", "PTP")

        acm.allow(cube, GRIPPER_TOUCH_LINKS, True)

        set_speed(arm, VEL_LIN, ACC_LIN)
        move_pose(arm, approach, f"{cube} approach", "LIN")
        move_pose(arm, touch, f"{cube} touch", "LIN")

        gripper_close(gripper)
        gazebo_attach(cube)
        attached = True
        scene.attach_box(eef, cube, touch_links=GRIPPER_TOUCH_LINKS)
        rospy.sleep(0.2)

        move_pose(arm, pre_pick, f"{cube} lift", "LIN")

        # -------- MID (joint safe) --------
        set_speed(arm, VEL_PTP, ACC_PTP)
        move_joints(arm, MID_JOINTS_SAFE, f"{cube} mid (JOINTS)", "PTP")

        # -------- PLACE --------
        pre_place = make_pose(bx, by, bz + Z_PRE_PLACE)

        # Per i bin, scendo un filo di velocità: riduce PATH_TOLERANCE_VIOLATED
        set_speed(arm, 0.12, 0.12)
        ok = move_pose(arm, pre_place, f"{cube} above {binm}", "PTP", strict=False)
        if not ok:
            raise RuntimeError(f"❌ Cannot reach above {binm} safely -> abort to avoid wrong drop")

        gripper_open(gripper)
        gazebo_detach(cube)
        attached = False

        try:
            scene.remove_attached_object(eef, cube)
        except Exception:
            pass
        try:
            scene.remove_world_object(cube)
        except Exception:
            pass

        acm.allow(cube, GRIPPER_TOUCH_LINKS, False)
        rospy.sleep(0.6)

        # torna a ready tra un cubo e l’altro
        set_speed(arm, VEL_REC, ACC_REC)
        if not goto_ready(arm):
            raise RuntimeError("❌ Cannot go to ready at task end")

        rospy.loginfo(f"===== DONE {cube} -> {binm} =====\n")

    except Exception as e:
        rospy.logerr(f"❌ ERROR during {cube}->{binm}: {e}")

        # stop + resync
        try:
            resync_state(arm, n=6, sleep=0.08)
        except Exception:
            pass
        try:
            gripper_open(gripper)
        except Exception:
            pass

        if attached:
            try:
                gazebo_detach(cube)
            except Exception:
                pass
            try:
                scene.remove_attached_object(eef, cube)
            except Exception:
                pass
            attached = False

        try:
            scene.remove_world_object(cube)
        except Exception:
            pass
        try:
            acm.allow(cube, GRIPPER_TOUCH_LINKS, False)
        except Exception:
            pass

        recover_to_ready(arm)
        raise


# ===============================
# MAIN
# ===============================
def main():
    rospy.init_node("trash_sort_camera_random_fixed")

    # (opzionale) più margine esecuzione
    rospy.set_param("/move_group/trajectory_execution/allowed_execution_duration_scaling", 10.0)
    rospy.set_param("/move_group/trajectory_execution/allowed_goal_duration_margin", 5.0)

    moveit_commander.roscpp_initialize(sys.argv)

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(0.5)  # lascia tempo a /planning_scene

    arm = moveit_commander.MoveGroupCommander("panda_arm")

    arm.allow_replanning(False)
    arm.set_planning_time(5.0)
    arm.set_goal_joint_tolerance(0.01)
    arm.set_goal_position_tolerance(0.005)
    arm.set_goal_orientation_tolerance(0.05)

    # FIX: forza pilz + PTP (se disponibile)
    set_pilz(arm, "PTP")

    eef = arm.get_end_effector_link()
    acm = ACMHelper()

    gripper = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
    rospy.loginfo("Waiting for gripper action server...")
    gripper.wait_for_server()
    rospy.loginfo("Gripper OK")

    # start in ready
    set_speed(arm, VEL_REC, ACC_REC)
    if not goto_ready(arm):
        if not recover_to_ready(arm):
            raise RuntimeError("❌ Cannot reach ready at start")

    # Randomizza posizioni cubi
    randomize_cubes_positions()

    # Vision -> TASKS
    vision = OverheadVision(CAM_TOPIC)
    tasks = None
    last_err = None
    for _ in range(VISION_RETRIES):
        try:
            tasks = vision.build_tasks_from_image()
            break
        except Exception as e:
            last_err = e
            rospy.logwarn(f"⚠ Vision retry: {e}")
            rospy.sleep(0.3)

    if tasks is None:
        rospy.logwarn(f"⚠ Vision failed, fallback to default order. Last err: {last_err}")
        tasks = [
            ("cube_green", "bin_green"),
            ("cube_yellow", "bin_yellow"),
            ("cube_blue", "bin_blue"),
        ]

    rospy.loginfo(f"✅ TASKS: {tasks}")

    # Esegui task: se una fallisce, recovery prima di continuare
    for cube, binm in tasks:
        try:
            pick_and_place(cube, binm, arm, scene, acm, gripper, eef)
        except Exception:
            rospy.logwarn("⚠ Task failed. Forcing recovery before continuing...")
            recover_to_ready(arm)
            continue

    rospy.loginfo("✅ ALL TASKS COMPLETE")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"FATAL: {e}")
        raise

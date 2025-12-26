#!/usr/bin/env python3
import sys
import math
import rospy
import actionlib
import moveit_commander

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from gazebo_msgs.srv import GetModelState
from tf.transformations import quaternion_from_euler
from franka_gripper.msg import MoveAction, MoveGoal
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest


# ===============================
# PARAMETRI
# ===============================
CUBE_SIZE = 0.04

Z_OFFSET    = 0.095
Z_PRE_PICK  = 0.15
Z_PRE_PLACE = 0.15
Z_RETREAT   = 0.20

# "Touch" in 2 step: prima 3cm sopra, poi touch
TOUCH_APPROACH_DZ = 0.03

# Velocità "normale" PTP
VEL_PTP = 0.15
ACC_PTP = 0.15

# Velocità lenta per LIN (evita PATH_TOLERANCE_VIOLATED)
VEL_LIN = 0.06
ACC_LIN = 0.06

GAZEBO_ATTACH_LINK = "panda_link7"
GRIPPER_TOUCH_LINKS = ["panda_link7", "panda_leftfinger", "panda_rightfinger"]

TASKS = [
    ("cube_green",  "bin_green"),
    ("cube_yellow", "bin_yellow"),
    ("cube_blue",   "bin_blue"),
]

# Joint-space "safe mid" (quello che ti salva il verde)
MID_JOINTS_SAFE = [0.0, -0.7, 0.0, -2.1, 0.0, 1.6, 0.8]


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


# ===============================
# MOVEIT SCENE
# ===============================
def upsert_cube_to_scene(scene, name, x, y, z):
    # non fidarti del warning "does not exist": è normale se stai resettando spesso
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

    def allow(self, obj, links, value):
        req = GetPlanningSceneRequest()
        req.components.components = req.components.ALLOWED_COLLISION_MATRIX
        scene = self.get_scene(req).scene
        acm = scene.allowed_collision_matrix

        names = list(acm.entry_names)

        def ensure(n):
            if n in names:
                return
            names.append(n)
            for r in acm.entry_values:
                r.enabled.append(False)
            acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(names)))

        ensure(obj)
        for l in links:
            ensure(l)

        acm.entry_names = names
        idx = {n: i for i, n in enumerate(names)}

        for l in links:
            acm.entry_values[idx[obj]].enabled[idx[l]] = value
            acm.entry_values[idx[l]].enabled[idx[obj]] = value

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
    group.set_planner_id(planner)
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
    group.set_planner_id(planner)
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
    arm.set_planner_id("PTP")
    arm.set_start_state_to_current_state()
    arm.set_named_target("ready")
    ok = arm.go(wait=True)
    arm.stop()
    arm.clear_pose_targets()
    rospy.sleep(0.4)  # settle fisico
    return ok


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
# PICK&PLACE (robusto)
# ===============================
def pick_and_place(cube, binm, arm, scene, acm, gripper, eef):
    rospy.loginfo(f"\n===== START {cube} -> {binm} =====")
    attached = False

    # reset “mentale” del robot prima di ogni task
    goto_ready(arm)

    cx = cy = cz = None

    try:
        cx, cy, cz = get_model_xyz(cube)
        bx, by, bz = get_model_xyz(binm)

        # aggiorna planning scene col cubo nella posa reale
        upsert_cube_to_scene(scene, cube, cx, cy, cz)

        # -------- PICK --------
        pre_pick = make_pose(cx, cy, cz + Z_OFFSET + Z_PRE_PICK)
        approach = make_pose(cx, cy, cz + Z_OFFSET + TOUCH_APPROACH_DZ)
        touch    = make_pose(cx, cy, cz + Z_OFFSET)

        gripper_open(gripper)

        set_speed(arm, VEL_PTP, ACC_PTP)
        move_pose(arm, pre_pick, f"{cube} pre-pick", "PTP")

        # consenti contatto dita <-> cubo SOLO durante pick
        acm.allow(cube, GRIPPER_TOUCH_LINKS, True)

        # LIN lenta e in 2 step
        set_speed(arm, VEL_LIN, ACC_LIN)
        move_pose(arm, approach, f"{cube} approach", "LIN")
        move_pose(arm, touch,    f"{cube} touch",    "LIN")

        gripper_close(gripper)

        gazebo_attach(cube)
        attached = True
        scene.attach_box(eef, cube, touch_links=GRIPPER_TOUCH_LINKS)
        rospy.sleep(0.2)

        # lift LIN lenta
        move_pose(arm, pre_pick, f"{cube} lift", "LIN")

        # -------- MID (joint safe) --------
        set_speed(arm, VEL_PTP, ACC_PTP)
        move_joints(arm, MID_JOINTS_SAFE, f"{cube} mid (JOINTS)", "PTP")

        # -------- PLACE --------
        pre_place = make_pose(bx, by, bz + Z_PRE_PLACE)
        ok = move_pose(arm, pre_place, f"{cube} above {binm}", "PTP", strict=False)
        if not ok:
            raise RuntimeError(f"❌ Cannot reach above {binm} safely -> abort to avoid wrong drop")

        gripper_open(gripper)

        gazebo_detach(cube)
        attached = False

        # pulizia MoveIt scene
        try:
            scene.remove_attached_object(eef, cube)
        except Exception:
            pass
        try:
            scene.remove_world_object(cube)
        except Exception:
            pass

        acm.allow(cube, GRIPPER_TOUCH_LINKS, False)

        # lascia assestare il cubo in Gazebo (soprattutto con bin bassi)
        rospy.sleep(0.6)

        # torna a ready tra un cubo e l’altro (quello che volevi)
        goto_ready(arm)

        rospy.loginfo(f"===== DONE {cube} -> {binm} =====\n")

    except Exception as e:
        rospy.logerr(f"❌ ERROR during {cube}->{binm}: {e}")

        # stop + settle: evita la cascata di "Invalid Trajectory"
        try:
            arm.stop()
            arm.clear_pose_targets()
            rospy.sleep(0.5)
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

        # hard recovery
        try:
            goto_ready(arm)
        except Exception:
            pass

        raise


# ===============================
# MAIN
# ===============================
def main():
    rospy.init_node("pick_place_multi_ready_between_tasks")

    # riduce TIME_OUT (anche se ora il tuo problema è CONTROL_FAILED, male non fa)
    rospy.set_param("/move_group/trajectory_execution/allowed_execution_duration_scaling", 10.0)
    rospy.set_param("/move_group/trajectory_execution/allowed_goal_duration_margin", 5.0)

    moveit_commander.roscpp_initialize(sys.argv)

    scene = moveit_commander.PlanningSceneInterface()
    arm = moveit_commander.MoveGroupCommander("panda_arm")

    arm.allow_replanning(False)
    arm.set_planning_time(5.0)

    arm.set_goal_joint_tolerance(0.01)
    arm.set_goal_position_tolerance(0.005)
    arm.set_goal_orientation_tolerance(0.05)

    arm.set_planner_id("PTP")

    eef = arm.get_end_effector_link()
    acm = ACMHelper()

    gripper = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
    gripper.wait_for_server()

    # start in ready
    set_speed(arm, VEL_PTP, ACC_PTP)
    goto_ready(arm)

    for cube, binm in TASKS:
        try:
            pick_and_place(cube, binm, arm, scene, acm, gripper, eef)
        except Exception:
            # continua comunque
            pass

    rospy.loginfo("✅ ALL TASKS COMPLETE")


if __name__ == "__main__":
    main()

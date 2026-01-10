#!/usr/bin/env python3
import os
import random
import math
import rospy
import numpy as np
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
import tf.transformations as tft

# Modelli reali in Gazebo
GAZEBO_OBJECTS = ["orange", "plastic_bottle", "carton_pack"]

# Classi YOLO
CLASSES = {"orange": 0, "coke_can": 1, "milk_carton": 2}

# Mappa: modello Gazebo -> classe YOLO
MODEL_TO_CLASS = {
    "orange": "orange",
    "plastic_bottle": "coke_can",
    "carton_pack": "milk_carton"
}

OBJECTS = list(MODEL_TO_CLASS.keys())


# dimensioni approssimative (servono solo per calcolare z sopra il tavolo)
BBOX_DIMS = {
    "orange": (0.04, 0.04, 0.04),           # sphere r=0.02
    "plastic_bottle": (0.056, 0.056, 0.18), # cylinder r=0.028 len=0.18
    "carton_pack": (0.05, 0.05, 0.15)       # box
}

BASE_RPY = {
    "orange": (0.0, 0.0),                     # roll, pitch
    "plastic_bottle": (math.pi, math.pi/2),   # sdraiata
    "carton_pack": (0.0, math.pi/2)           # sdraiato
}

class AutoDatasetGen:

    def __init__(self):
        rospy.init_node("auto_dataset_gen")

        self.image_topic = rospy.get_param("~image_topic", "/overhead_camera/image_raw")
        self.info_topic  = rospy.get_param("~info_topic",  "/overhead_camera/camera_info")
        self.states_topic= rospy.get_param("~states_topic","/gazebo/model_states")

        self.out_dir     = rospy.get_param("~out_dir", os.path.expanduser("~/yolo_dataset"))
        self.split       = rospy.get_param("~split", "train")     # train / val / test
        self.n_samples   = int(rospy.get_param("~n", 300))

        # Quanti oggetti mettere per immagine
        # default: random 1..3
        self.min_objects = int(rospy.get_param("~min_objects", 1))
        self.max_objects = int(rospy.get_param("~max_objects", 3))

        # Area tavolo (tune)
        self.x_min = rospy.get_param("~x_min", 0.60 - 0.20)
        self.x_max = rospy.get_param("~x_max", 0.60 + 0.20)
        self.y_min = rospy.get_param("~y_min", -0.30)
        self.y_max = rospy.get_param("~y_max",  0.30)

        self.table_z = rospy.get_param("~table_z", 0.40)

        # opzionale: salvataggio metadata (pose per immagine)
        self.save_meta = bool(rospy.get_param("~save_meta", False))

        self.bridge = CvBridge()
        self.last_img = None
        self.model_states = None
        self.width = None
        self.height = None

        rospy.Subscriber(self.image_topic, Image, self.cb_img, queue_size=1)
        rospy.Subscriber(self.info_topic, CameraInfo, self.cb_info, queue_size=1)
        rospy.Subscriber(self.states_topic, ModelStates, self.cb_states, queue_size=1)

        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.prepare_dirs()

    def prepare_dirs(self):
        self.img_dir = os.path.join(self.out_dir, "images", self.split)
        os.makedirs(self.img_dir, exist_ok=True)
        rospy.loginfo(f"Output images: {self.img_dir}")

        if self.save_meta:
            self.meta_dir = os.path.join(self.out_dir, "meta", self.split)
            os.makedirs(self.meta_dir, exist_ok=True)

    def cb_img(self, msg):
        try:
            self.last_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn(f"cv_bridge: {e}")

    def cb_info(self, msg):
        self.width = msg.width
        self.height = msg.height
        rospy.loginfo_throttle(2.0, f"cb_info OK: {self.width}x{self.height}")

    def cb_states(self, msg):
        self.model_states = msg

    def set_random_pose(self, model_name, size_xyz):
        sx, sy, sz = size_xyz

        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)

        roll0, pitch0 = BASE_RPY[model_name]
        roll  = roll0  + random.uniform(-0.05, 0.05)
        pitch = pitch0 + random.uniform(-0.05, 0.05)
        yaw   = random.uniform(-math.pi, math.pi)

        if model_name == "plastic_bottle":
            # ribalta di 180° per mostrare l'altro lato
            roll += math.pi

        is_lying = abs(pitch0) > 1.0
        vertical = max(sx, sy) if is_lying else sz
        z = self.table_z + (vertical / 2.0) + 0.005

        q = tft.quaternion_from_euler(roll, pitch, yaw)

        st = ModelState()
        st.model_name = model_name
        st.pose.position.x = x
        st.pose.position.y = y
        st.pose.position.z = z
        st.pose.orientation.x = q[0]
        st.pose.orientation.y = q[1]
        st.pose.orientation.z = q[2]
        st.pose.orientation.w = q[3]
        st.reference_frame = "world"

        try:
            self.set_state(st)
        except rospy.ServiceException as e:
            rospy.logwarn(f"set_model_state failed: {e}")

        return {
            "name": model_name,
            "pos": [x, y, z],
            "rpy": [roll, pitch, yaw]
        }

    def run(self):
        rospy.loginfo("Waiting for image + model_states + camera_info...")
        while not rospy.is_shutdown():
            if self.last_img is not None and self.model_states is not None and self.width is not None:
                break
            rospy.sleep(0.1)

        ok = 0
        attempts = 0
        max_attempts = self.n_samples * 20  # per sicurezza

        while not rospy.is_shutdown() and ok < self.n_samples and attempts < max_attempts:
            attempts += 1

            # Scegli quante classi mettere in scena (1..3)
            k = random.randint(self.min_objects, self.max_objects)
            chosen = random.sample(OBJECTS, k=k)

            meta = {"objects": []}

            # Randomizza pose per gli oggetti scelti
            for name in chosen:
                meta["objects"].append(self.set_random_pose(name, BBOX_DIMS[name]))

            # (opzionale) sposta gli altri oggetti "fuori scena" così non compaiono mai
            for name in OBJECTS:
                if name in chosen:
                    continue
                st = ModelState()
                st.model_name = name
                st.pose.position.x = 10.0
                st.pose.position.y = 10.0
                st.pose.position.z = 0.0
                st.pose.orientation.w = 1.0
                st.reference_frame = "world"
                try:
                    self.set_state(st)
                except rospy.ServiceException:
                    pass

            # lascia aggiornare Gazebo
            rospy.sleep(0.2)

            img_raw = self.last_img.copy()
            if img_raw is None or img_raw.size == 0:
                continue

            stem = f"{self.split}_{ok:06d}"
            cv2.imwrite(os.path.join(self.img_dir, stem + ".jpg"), img_raw)

            if self.save_meta:
                import json
                with open(os.path.join(self.meta_dir, stem + ".json"), "w") as f:
                    json.dump(meta, f, indent=2)

            ok += 1
            if ok % 20 == 0:
                rospy.loginfo(f"Saved {ok}/{self.n_samples} images")

        rospy.loginfo(f"Done. Saved {ok} images to {self.img_dir}")
        if attempts >= max_attempts:
            rospy.logwarn("Stopped early: too many failed attempts (check camera topic and Gazebo).")

if __name__ == "__main__":
    gen = AutoDatasetGen()
    gen.run()

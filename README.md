# trash-sorter-SR (ROS Noetic + Gazebo Classic + MoveIt)

Progetto: Franka Panda in simulazione (Gazebo) prende 3 cubi colorati (verde/giallo/blu) e li deposita nel bin del colore corrispondente.  
L’ordine dei cubi viene rilevato tramite una camera RGB dall’alto (ordinamento **left-to-right** nell’immagine).

Questa repo contiene **SOLO** il package ROS `trash_sorter`.  
Tutte le altre dipendenze vanno installate/clonate nel workspace.

---

## Ambiente testato
- Ubuntu 20.04
- ROS Noetic
- Gazebo Classic (gazebo11 su 20.04)
- MoveIt (MoveIt1)
- `catkin_tools` (`catkin build`)
- OpenCV + `cv_bridge` (per visione colore dall’overhead camera)

---

## Struttura prevista del workspace

Workspace consigliato (uguale a quello usato nello sviluppo):
~/franka_ws/
src/
franka_ros/
panda_moveit_config/
gazebo_ros_link_attacher/
moveit/ (opzionale: solo se compili MoveIt da sorgente)
moveit_msgs/ (opzionale: come sopra)
moveit_resources/ (opzionale: come sopra)
moveit_tutorials/ (opzionale: come sopra)
rviz_visual_tools/ (opzionale: come sopra)
moveit_visual_tools/ (opzionale: come sopra)
srdfdom/ (opzionale: come sopra)
geometric_shapes/ (opzionale: come sopra)
position_tracker/ (opzionale: se ti serve, altrimenti no)
trash_sorter/ (QUESTA REPO)


# A) Installazione (consigliata): MoveIt via APT (più semplice e riproducibile)

## 0) Prerequisiti: ROS + catkin_tools
bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  python3-catkin-tools \
  python3-rosdep \
  git
sudo rosdep init 2>/dev/null || true
rosdep update

Aggiungi ROS al ~/.bashrc (se non c’è già):
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

## 1) Crea workspace
mkdir -p ~/franka_ws/src
cd ~/franka_ws
catkin init

## 2) Dipendenze APT (MoveIt + Gazebo ROS + OpenCV bridge)
sudo apt update
sudo apt install -y \
  ros-noetic-moveit \
  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-cv-bridge ros-noetic-image-transport \
  python3-opencv python3-numpy

## 3) Clona dipendenze (quelle DAVVERO necessarie) in src/
cd ~/franka_ws/src

# franka_ros: include franka_gazebo + controller + Panda in Gazebo
git clone https://github.com/frankaemika/franka_ros.git

# gazebo link attacher: per attaccare/staccare i cubi al gripper in Gazebo
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git

## 4) Clona questa repo (trash_sorter) nel workspace
cd ~/franka_ws/src
git clone https://github.com/Rocco-Meoli/trash-sorter-SR.git

# questa repo contiene una cartella trash_sorter/ già pronta: copiala nel workspace
cp -r trash-sorter-SR/trash_sorter ~/franka_ws/src/

# permessi esecuzione script python
chmod +x ~/franka_ws/src/trash_sorter/scripts/*.py

## 5) (Fondamentale) Modifica manuale controller (senza patch)

Per ridurre abort intermittenti del controller tipo:

GOAL_TOLERANCE_VIOLATED

CONTROL_FAILED

modifica manualmente questo file:
~/franka_ws/src/franka_ros/franka_gazebo/config/sim_controllers.yaml
Trova il blocco:
position_joint_trajectory_controller:

e assicurati che le constraints siano queste (valori usati nel progetto):
Trova il blocco:
position_joint_trajectory_controller:
position_joint_trajectory_controller:
  type: position_controllers/JointTrajectoryController
  joints:
    - $(arg arm_id)_joint1
    - $(arg arm_id)_joint2
    - $(arg arm_id)_joint3
    - $(arg arm_id)_joint4
    - $(arg arm_id)_joint5
    - $(arg arm_id)_joint6
    - $(arg arm_id)_joint7
  constraints:
    goal_time: 2.0
    stopped_velocity_tolerance: 0.05
    $(arg arm_id)_joint1: { goal: 0.12, trajectory: 0.30 }
    $(arg arm_id)_joint2: { goal: 0.12, trajectory: 0.30 }
    $(arg arm_id)_joint3: { goal: 0.12, trajectory: 0.30 }
    $(arg arm_id)_joint4: { goal: 0.12, trajectory: 0.30 }
    $(arg arm_id)_joint5: { goal: 0.12, trajectory: 0.30 }
    $(arg arm_id)_joint6: { goal: 0.12, trajectory: 0.30 }
    $(arg arm_id)_joint7: { goal: 0.12, trajectory: 0.30 }

## 6) Installa dipendenze con rosdep + build
cd ~/franka_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash

Consiglio pratico (per non impazzire ogni terminale):
bash
Copy code
echo "source ~/franka_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

## 7) Avvio simulazione (Gazebo + Panda + MoveIt)
Il launch del package usa:
roslaunch trash_sorter demo_gazebo_trash_world_arg.launch pipeline:=pilz_industrial_motion_planner

## 8) Avvio script sorting
In un nuovo terminale:
source ~/franka_ws/devel/setup.bash
rosrun trash_sorter move_test.py

# trash-sorter-SR (ROS Noetic + Gazebo + MoveIt)

Progetto: Franka Panda prende 3 cubi colorati (verde/giallo/blu) e li deposita nel contenitore (bin) del colore corrispondente.
L’ordine dei cubi viene rilevato tramite una camera RGB dall’alto (ordinamento left-to-right).

Questa repo contiene SOLO il package ROS `trash_sorter`.
Le dipendenze (franka_ros, MoveIt, gazebo_ros_link_attacher, ecc.) si installano/clonano separatamente.

---

## Ambiente testato
- Ubuntu 20.04
- ROS Noetic
- Gazebo classic
- MoveIt
- franka_ros (sim in Gazebo)
- catkin_tools (`catkin build`)

---

## 0) Prerequisiti
Installa ROS Noetic e catkin_tools se non li hai già.

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  python3-catkin-tools \
  python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

Apri il tuo .bashrc e assicurati di avere:
source /opt/ros/noetic/setup.bash

1) Crea workspace (catkin build)
mkdir -p ~/franka_ws/src
cd ~/franka_ws
catkin init

2) Dipendenze APT (MoveIt, Gazebo, OpenCV, bridge ROS)
sudo apt update
sudo apt install -y \
  ros-noetic-moveit \
  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-cv-bridge ros-noetic-image-transport \
  python3-opencv python3-numpy

3) Clona dipendenze nella cartella src
cd ~/franka_ws/src

# franka_ros (simulazione Panda in Gazebo)
git clone https://github.com/frankaemika/franka_ros.git

# gazebo link attacher (serve per "attaccare" e "staccare" i cubi al gripper in Gazebo)
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git

4) Clona questa repo e aggiungi il package trash_sorter

NOTA: il workspace deve contenere una cartella trash_sorter/ (package ROS).
Questa repo è fatta apposta per essere copiata dentro ~/franka_ws/src/.

cd ~/franka_ws/src
git clone https://github.com/Rocco-Meoli/trash-sorter-SR.git

# Copia il package vero e proprio dentro il workspace
cp -r trash-sorter-SR/trash_sorter ~/franka_ws/src/

# Permessi esecuzione sugli script
chmod +x ~/franka_ws/src/trash_sorter/scripts/*.py

5) IMPORTANTISSIMO: modifica manuale controller (senza patch)

Per ridurre abort intermittenti tipo:

GOAL_TOLERANCE_VIOLATED

CONTROL_FAILED

modifica manualmente:

~/franka_ws/src/franka_ros/franka_gazebo/config/sim_controllers.yaml

Cerca il blocco:
position_joint_trajectory_controller:

e imposta queste constraints (valori usati in questo progetto):
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

6) Installa dipendenze con rosdep e builda
cd ~/franka_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash

Consiglio: aggiungi al .bashrc (così non lo fai a mano ogni volta):

echo "source ~/franka_ws/devel/setup.bash" >> ~/.bashrc

7) Avvio simulazione (Gazebo + Panda + MoveIt)
Consigliato (super stabile): world senza tavoli
roslaunch trash_sorter demo_gazebo_trash_world_arg.launch pipeline:=pilz_industrial_motion_planner

8) Avvio script sorting

In un nuovo terminale:

source ~/franka_ws/devel/setup.bash
rosrun trash_sorter move_test.py

Topic / Service principali
Overhead Camera (ordine cubi)

Topic immagine:

/overhead_camera/image_raw

Gazebo link attacher (attach/detach cubi)

Service:

/link_attacher_node/attach

/link_attacher_node/detach

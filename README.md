# RoboLink-dci-5s-m-ros2-v13

This repository provides a ROS 2 Humble workspace for the RL-DCi-5S-M robotic arm by Commonplace Robotics, specifically for firmware version **v13 or lower**.

It includes packages for description, hardware interface, MoveIt 2 integration, controller nodes, and usage examples.

---

## 📦 Included Packages

- `cobot_description`: URDF/XACRO, meshes, and RViz setup
- `cobot_hardware`: Interface to physical robot via CRI protocol
- `cobot_controller`: ROS 2 controllers (e.g. JointTrajectory)
- `cobot_moveit_config`: MoveIt 2 planning configuration
- `cobot_examples`: Example scripts and launch files
- `cobot_calibration`: Eye-in-hand camera calibration tools

---

## ⚙️ Requirements

- **ROS 2 Humble**
- **Python ≥ 3.10**
- [`cri-py-v13-compatible`](https://github.com/TengSudo/cri-py-v13-compatible) Python library (custom fork)
- Optional: Realsense SDK (for Intel D435 3D camera)

---

## 🔧 Network Configuration

Before connecting to the RL-DCi-5S-M:

1. Set your PC's **IPv4 address** to:  
   `192.168.3.1`

2. Connect the robot's Ethernet to your PC directly or via a router.

---

## 📦 Installing Dependencies

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions ros-humble-rviz2 ros-humble-joint-state-publisher-gui

# Clone CRI library
git clone https://github.com/TengSudo/cri-py-v13-compatible.git
cd cri-py-v13-compatible
pip install .

## 📄 License

This project is for research and educational purposes.

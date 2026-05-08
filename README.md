# 🤖 Robot Simulation

A ROS2 differential drive robot simulation, built on **ROS2 Humble** and **Ignition Fortress (Gazebo 6.17)**.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Ignition%20Fortress-orange)

---

## 📷 Preview

 ![Move](images/sim_rv_gz.gif)

<img src="images/camera_lidar.png" width="800"/>

<p align="left">
  <img src="images/multi_robot_1.png" width="400"/>
  <img src="images/multi_robot_2.png" width="400"/>
</p>

<img src="images/frames.png" width="800"/>

---

## 📦 Package Structure

```
robot_ws/
└── src/
    ├── images/                     # Images, GIF
    │
    ├── robot_bringup/              # Launch files, configs
    │   ├── config/
    │   │   └── robot.rviz
    │   ├── launch/
    │   │   └── robot.launch.py
    │   └── scripts/
    │       └── teleop_keyboard.py
    │
    ├── robot_description/          # URDF/Xacro, meshes
    │   ├── hooks/
    │   ├── meshes/
    │   └── urdf/
    │       └── robot.xacro
    │
    └── robot_gazebo/               # Simulation worlds
        ├── hooks/
        └── worlds/
            └── demo.sdf
```

---

## 🤖 Robot Specifications

|  Parameter           | Value      |
|:---------------------|-----------:|
|  Wheel Radius        | 0.0792 m   |
|  Wheel Separation    | 0.288 m    |
|  Max Linear Velocity | 1.0 m/s    |
|  Max Angular Velocity| 1.0 rad/s  |

### 📡 Sensors

|  Sensor       |   Type           |   Topic         | Rate   |
|:--------------|:-----------------|:----------------|-------:|
|  IMU          |   9-DOF IMU      |   `/r1/imu`     | 50 Hz  |
|  LiDAR        |   Hokuyo UST-10  |   `/r1/lidar`   | 10 Hz  |
|  Depth Camera | Intel RealSense D435 | `/camera/*` | 5 Hz   |

---

## 🔧 Prerequisites

- **OS:** Ubuntu 22.04
- **ROS2:** Humble
- **Gazebo:** Ignition Fortress

Install ROS2 Humble by following the [official guide](https://docs.ros.org/en/humble/Installation.html).

Install Ignition Fortress:
```bash
sudo apt-get install ignition-fortress
```

Install ROS-Gazebo bridge:
```bash
sudo apt install ros-humble-ros-gz
```

---

## 🚀 Installation

**1. Create workspace and clone the repository:**
```bash
mkdir -p ~/robot_ws/src && cd ~/robot_ws/src
git clone https://github.com/ngducdatRb/ROS2-Autonomous-Mobile-Robot-Simulation.git
```

**2. Install dependencies:**
```bash
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
```

**3. Build:**
```bash
colcon build
source install/setup.bash
```

---

## ▶️ Usage

### Launch Simulation

```bash
ros2 launch robot_bringup simulation.launch.py
```

This will start:
- **Ignition Fortress** with the demo world
- **Robot State Publisher** — publishes TF transforms
- **ROS-GZ Bridge** — bridges topics between Gazebo and ROS2
- **RViz2** — visualization

### Teleoperate the Robot

```bash
# Move to teleop script folder
cd ~/robot_ws/src/robot_bringup/scripts

# Run teleop node
python3 teleop_keyboard.py
```

### Monitor Topics

```bash
# Check odometry
ros2 topic echo /r1/odom

# Check LiDAR
ros2 topic echo /r1/lidar

# Check IMU
ros2 topic echo /r1/imu
```
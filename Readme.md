# UAV Simulation with PX4, ROS2 & Zenoh

Environment for running UAV simulations with **PX4**, **ROS2**, **Gazebo**, and **Zenoh**
---

## Features

- (on-going)Fully containerized PX4 SITL environment for Ubuntu 22.04


## Components Included

| Component           | Version                      | Description                           |
| ------------------- | ---------------------------- | ------------------------------------- |
| **Ubuntu**          | 22.04                        | Base OS                               |
| **ROS 2**           | Humble Hawksbill             | Core ROS2 environment                 |
| **PX4**             | v1.15.2                      | Autopilot firmware                    |
| **Gazebo Sim**      | Garden                       | Simulation environment                |
| **RMW_Zenoh**       | Latest                       | A ROS 2 RMW based on Zenoh            |
| **PX4 ROS2 Bridge** | 1.15                         | Includes `px4_msgs` and `px4_ros_com` |
| **Tools**           | tmux, nano, colcon           | Development & debugging               |

---

<details>
<summary>Frameworks & Tools Used</summary>

![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?logo=ubuntu)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros)
![PX4](https://img.shields.io/badge/PX4-v1.15.0-lightgrey?logo=px4)
![Gazebo](https://img.shields.io/badge/Gazebo-Garden-green?logo=gazebo)
![Docker](https://img.shields.io/badge/Docker-ready-blue?logo=docker)

</details>


## Directory Structure

```
px4_1.15_ros2_humble_ubuntu22.04/
├── build.sh
├── Dockerfile
├── entrypoint.sh
├── Readme.txt
├── run.sh
└── launch_files/
    └── camera_detection.sh
```

---

## Installation 

### Clone and build

```bash
git clone https://github.com/rrajnidhi/px4_1.15_ros2_humble_ubuntu22.04.git
cd px4_1.15_ros2_humble_ubuntu22.04
git switch zenoh_implementation
./build.sh
```

---

## Launch container

To start and enter the docker container 

```bash
./run_for_multi_UAV.sh
```

---

## A. Single drone SITL(without Mission)

Once inside to launch PX4 SITL + ROS2 Bridge:

```bash
cd /launch_files
./camera_detection.sh
```
This script launches a **tmux** session with:

| Pane | Description                                  |
| ---- | -------------------------------------------- |
| 0    | A Zenoh router                               |
| 1    | PX4 SITL + Gazebo (headless)                 |
| 2    | px4_ros2 bridge                              |

 
---

## Help with ros2 and tmux  

### Tmux Controls

* Detach: `Ctrl+b d`
* Move-between-panes: `Ctrl+b <arrow-key>`
* Reattach:

```bash
tmux attach -t sitl_px4_ros2_humble
```

* Kill session:

```bash
Ctrl+b e
```

---

### List PX4 topics

```bash
ros2 topic list
```

### Echo vehicle odometry

```bash
ros2 topic echo /fmu/out/vehicle_odometry
```

## Cleanup

To stop and remove containers:

```bash
docker ps
docker stop <container_id>
```

The container is automatically removed on exit because `--rm` is used in `run.sh`.

---

## Maintainer

**Nidhi Raj**
[nidhirajr@gmail.com](mailto:nidhirajr@gmail.com)

---

## Acknowledgements

* [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
* [ROS 2 Humble](https://docs.ros.org/en/humble/)
* [Gazebo Sim](https://gazebosim.org/)

---


> Enjoy! 

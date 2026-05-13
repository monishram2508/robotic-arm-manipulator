# UR5 Pick and Place with Artificial Potential Fields

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-22314E?style=flat&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=flat&logo=python&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-00599C?style=flat&logo=cplusplus&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-enabled-2496ED?style=flat&logo=docker&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-FF6600?style=flat)

A ROS2-based simulation of a UR5 robotic arm performing pick-and-place operations using Artificial Potential Field (APF) local planning, with MoveIt2 for motion planning and Gazebo for simulation.

## Overview

The robot arm identifies a pre-planned object, navigates to it using APF-based local planning to avoid obstacles, picks it up, and places it at a target location. The system integrates:

- **APF Planner** — local planning using attractive and repulsive potential fields
- **MoveIt2** — motion planning and execution
- **Gazebo** — physics simulation with camera and Robotiq gripper
- **Docker** — containerized environment for reproducibility

## Requirements

- Docker
- ROS2 Humble

## Setup and Usage

All setup instructions, dependencies, launch steps, and usage details are documented in the [project report](report.pdf).

## Repository Structure

```
ur5_docker/
├── docker/          # Dockerfile and entrypoint
├── deps.repos       # External dependencies (vcs)
└── src/
    ├── apf_planner/                        # APF local planner nodes
    ├── ur5_simulation/                     # URDF, worlds, Gazebo config
    ├── ur5_camera_gripper_moveit_config/   # MoveIt2 configuration
    └── IFRA_LinkAttacher/                  # Gazebo link attacher plugin
```

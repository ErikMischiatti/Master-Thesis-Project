# Master Thesis Project - Robotic Manipulation with Learning from Demonstration (Status: Ongoing...)

This repository contains the code and resources for my Master Thesis project developed at the Technical University of Vienna (TUW) as part of the Mechatronic Engineering Master's program at the University of Trento. The project involves the use of a **Franka Emika Panda** robotic arm and an **Intel RealSense D435i** camera to implement a system for robotic manipulation tasks based on **demonstration learning**.

## Table of Contents
1. [Project Overview](#project-overview)
2. [Requirements](#requirements)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Project Structure](#project-structure)
6. [Future Work](#future-work)
7. [Acknowledgements](#acknowledgements)

## Project Overview

In this project, I aim to teach a robotic arm how to perform complex tasks using demonstration learning techniques. The key components include:
- Unified 6D Pose Estimation for object tracking.
- Behavior Trees for task execution.
- Imitation learning algorithms to reverse previously learned tasks.
- Modular communication system using **Ultra-Wideband (UWB)** technology (as described in my co-authored paper).
  
The system is designed to manipulate objects by performing a sequence of predefined trajectories, detect objects like cables, and adapt the trajectories based on sensory input. Through the use of demonstration learning, the robot can learn from human demonstrations and apply those learned skills in novel situations.

The project is split into several stages:
- **Learning from Demonstration (LfD)**: Capture human demonstrations using both the robot and the Intel RealSense camera.
- **Behavior Tree Integration**: Implement task sequences through a modular approach using behavior trees for robust decision-making and execution of robotic tasks.
- **Object Tracking and Pose Estimation**: Use 6D pose estimation for real-time object tracking to adjust robotic trajectories dynamically.
- **Trajectory Execution**: Use both predefined and learned trajectories to manipulate objects in the environment, adapting to changes detected through sensory input.

The robotic arm is controlled using **ROS** (Robot Operating System), which facilitates the communication between the different components and subsystems, such as the camera, sensors, and the robotic arm.

## Requirements

To run this project, you need the following:

### Hardware
- **Franka Emika Panda** robotic arm.
- **Intel RealSense D435i** camera for depth and RGB-D vision.
- **PC** running Ubuntu 20.04 (or later) with a ROS-compatible environment.

### Software
- **ROS Noetic** (or ROS 1 distribution compatible with your system).
- **Franka Control Interface (libfranka)** for controlling the robotic arm.
- **Intel RealSense SDK** for integrating the camera.
- **Python 3.8+**.
- **Behavior Trees Libraries** (PyTrees or BehaviorTree.CPP).
- Python dependencies (can be installed with `rosdep`).

### ROS Packages
You will need to install the following ROS packages:
- `franka_ros`: ROS interface for controlling Franka Emika Panda.
- `realsense-ros`: ROS drivers for Intel RealSense cameras.
- `moveit`: For motion planning and control of the robotic arm.

# Installation


# Expected project structure:


# Future Work:


# Acknowledgements:
 I would like to express my gratitude to:
 - My supervisor and the Phd students at the Technical University of Vienna (TUW) for their guidance and feedback.
 - The University of Trento for the education and support provided during my Mechatronic Engineering studies.


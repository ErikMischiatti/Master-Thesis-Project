# dhb_ros

## Overview

dhb_ros is a ROS package for implementing and executing movements based on the Denavit-Hartenberg Bidirectional (DHB) for robots. This package allows for learning and reproducing trajectories using the DHB representation.

## Requirements

- ROS (Robot Operating System) Noetic or later
- Python 3
- [Franka ROS](https://frankaemika.github.io/docs/installation_linux.html) for interfacing with the Franka Emika robot

## Installation

1. Clone the repository into your ROS workspace:
        cd ~/catkin_ws/src
    git clone <your_repository_url> dhb_ros
    cd ~/catkin_ws
    catkin_make
    

2. Source the setup.bash of your ROS workspace:
        source ~/catkin_ws/devel/setup.bash
    

## Usage

### Learning a DHB Trajectory

To learn a DHB trajectory from a data file:
1. Record the desired trajectory data and save it to a .npz file.
2. Use the dhb_learn_npz.py script to train the DHB model:
        rosrun dhb_ros dhb_learn_npz.py /path/to/your/data.npz
    

### Executing a DHB Trajectory

1. Edit the PlayDHB.launch file to specify the name of the trajectory to execute:
        <arg name="motion" default="your_trajectory_name_dhb" />
    

2. Launch the file to execute the trajectory:
        roslaunch dhb_ros PlayDHB.launch
    

### Trajectory Example

Example usage for learning and executing a trajectory:
1. Record a trajectory and save the data in example_trajectory.npz.
2. Learn the trajectory using the dhb_learn_npz.py script:
        rosrun dhb_ros dhb_learn_npz.py /path/to/example_trajectory.npz
    

3. Execute the trajectory using the launch file:
        roslaunch dhb_ros PlayDHB.launch motion:=example_trajectory_dhb
    

## Contributions

Contributions and pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License

This package is licensed under the MIT License - see the LICENSE file for details.
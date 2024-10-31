#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.rotations import matrix_from_quaternion

# Funzione per convertire una matrice di rotazione in angoli di Eulero (XYZ)
def euler_from_rotation_matrix(R):
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

# Funzione per convertire quaternioni in angoli di Eulero utilizzando la matrice di rotazione
def quaternions_to_euler(quaternions):
    euler_angles = np.array([euler_from_rotation_matrix(matrix_from_quaternion(q)) for q in quaternions])
    return euler_angles

# Function to plot poses (X, Y, Z axes) and orientations (roll, pitch, yaw) over time
def plot_poses_and_orientations(poses, orientations_quat, time_stamps, label_prefix):
    # Convert quaternions to Euler angles
    orientations_euler = quaternions_to_euler(orientations_quat)
    
    fig, axs = plt.subplots(6, 1, figsize=(10, 12))

    # Plot X, Y, Z positions
    axs[0].plot(time_stamps, poses[:, 0], label=f'{label_prefix} X')
    axs[0].set_title(f'{label_prefix} Position X over time')
    axs[0].set_xlabel('Time')
    axs[0].set_ylabel('X Position')

    axs[1].plot(time_stamps, poses[:, 1], label=f'{label_prefix} Y')
    axs[1].set_title(f'{label_prefix} Position Y over time')
    axs[1].set_xlabel('Time')
    axs[1].set_ylabel('Y Position')

    axs[2].plot(time_stamps, poses[:, 2], label=f'{label_prefix} Z')
    axs[2].set_title(f'{label_prefix} Position Z over time')
    axs[2].set_xlabel('Time')
    axs[2].set_ylabel('Z Position')

    # Plot roll, pitch, yaw (orientations)
    axs[3].plot(time_stamps, np.unwrap(orientations_euler[:, 0]), label=f'{label_prefix} Roll')
    axs[3].set_title(f'{label_prefix} Orientation Roll over time')
    axs[3].set_xlabel('Time')
    axs[3].set_ylabel('Roll')

    axs[4].plot(time_stamps, orientations_euler[:, 1], label=f'{label_prefix} Pitch')
    axs[4].set_title(f'{label_prefix} Orientation Pitch over time')
    axs[4].set_xlabel('Time')
    axs[4].set_ylabel('Pitch')

    axs[5].plot(time_stamps, orientations_euler[:, 2], label=f'{label_prefix} Yaw')
    axs[5].set_title(f'{label_prefix} Orientation Yaw over time')
    axs[5].set_xlabel('Time')
    axs[5].set_ylabel('Yaw')

    plt.tight_layout()
    plt.show()

# Load the files
data_2 = np.load('/home/asl_team/catkin_ws/src/dhb_ros-General/data/dhb/generated_traj_dhb_vel_approach_clip_3_ee_velocities.npz')
data_1 = np.load('/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/approach_clip_3_ee_velocities.npz')

# Extracting data from the first file for poses and time
poses_1 = data_1['poses']
time_stamps_1 = data_1['time_stamps']

# Extracting data from the second file for positions and orientations (quaternions)
positions_2 = data_2['positions']
orientations_2 = data_2['orientations']
duration_2 = data_2['duration']

# Truncate the longer array (time_stamps_1) to match the length of the orientations
min_length = min(len(time_stamps_1), len(orientations_2))
time_stamps_1 = time_stamps_1[:min_length]
poses_1 = poses_1[:min_length]

# Plotting poses and orientations for the first file
plot_poses_and_orientations(poses_1, orientations_2, time_stamps_1, 'File 1')

# Simulating the time array for the second file based on duration and length of positions
time_stamps_2 = np.linspace(0, duration_2, len(positions_2))

# Plotting poses and orientations for the second file
plot_poses_and_orientations(positions_2, orientations_2, time_stamps_2, 'File 2')

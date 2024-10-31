#!/usr/bin/env python3
import numpy as np
import os
from movement_primitives.dmp import CartesianDMP
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def load_data(file_name):
    file_path = os.path.join(BASE_DIR, 'data', file_name)
    return np.load(file_path)

def train_dmp(pose_traj, time, n_bfs):
    duration = time[-1]
    dt = np.mean(np.diff(time))

    dmp_segment = CartesianDMP(execution_time=duration, dt=dt, n_weights_per_dim=n_bfs, smooth_scaling=True)
    dmp_segment.imitate(time, pose_traj)

    weights = dmp_segment.get_weights()

    return weights, duration

def dmp_gen_traj(weights, duration, start, goal):
    n_bfs = int(weights.shape[0]/6)
    dmp = CartesianDMP(execution_time=duration, dt=0.05, n_weights_per_dim=n_bfs, smooth_scaling=True)
    dmp.set_weights(weights)
    dmp.configure(start_y=start, goal_y=goal)
    dmp_time, dmp_pose = dmp.open_loop()

    return dmp_time, dmp_pose

if __name__ == '__main__':
    file_name = 'BuildBot_recordings/grab_leg/grab_leg_6.npz'
    plot = True
    
    # Load data
    fps = 15
    data = load_data(file_name)
    pose_traj = data['pose_quat_log_f'].T
    pose_traj= pose_traj[:, [0, 1, 2, 6, 3, 4, 5]]
    duration = pose_traj.shape[0]/fps
    time = np.linspace(0, duration, pose_traj.shape[0])
    n_bfs = 5
    
    # Train DMP
    weights, duration = train_dmp(pose_traj, time, n_bfs)

    # Define save directory and file name
    save_dir = os.path.join(BASE_DIR, 'data', 'dmp')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    base_file_name = os.path.splitext(os.path.basename(file_name))[0] + '_dmp'
    save_path = os.path.join(save_dir, base_file_name + '.npz')
    np.savez(save_path, weights=weights, duration=duration, demo_goal=pose_traj[-1, :])

    print("DMP training completed and data saved to", save_path)

    # Plot trajectory if required
    if plot:
        dmp_time, dmp_pose = dmp_gen_traj(weights, duration, start=pose_traj[0,:], goal=pose_traj[-1,:])

        print(f"final error {np.linalg.norm(pose_traj[-1,:3]-dmp_pose[-1,:3])}")
        print(f"error: {pose_traj[-1,:3]-dmp_pose[-1,:3]}")

        fig, axs = plt.subplots(7, 1, figsize=(10, 15))
        labels = ['X position', 'Y position', 'Z position', 'Quaternion w', 'Quaternion x', 'Quaternion y', 'Quaternion z']
        for i, ax in enumerate(axs):
            ax.plot(time, pose_traj[:, i], '-', label='Original')
            ax.plot(dmp_time, dmp_pose[:, i], '--', label='DMP')
            ax.set_ylabel(labels[i])
            ax.set_xlabel('Time (s)')
            ax.legend()
            ax.grid(True)

        # Adjust layout to prevent overlap
        plt.tight_layout()
        plt.show()

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(pose_traj[:, 0], pose_traj[:, 1], pose_traj[:, 2], '-', label='demo')
        ax.plot(dmp_pose[:, 0], dmp_pose[:, 1], dmp_pose[:, 2], '--', label='dmp')

        ax.plot(pose_traj[0, 0], pose_traj[0, 1], pose_traj[0, 2], 'o', color="green", label='demo start')
        ax.plot(pose_traj[-1, 0], pose_traj[-1, 1], pose_traj[-1, 2], 'o', color="red", label='demo end')

        ax.plot(dmp_pose[0, 0], dmp_pose[0, 1], dmp_pose[0, 2], '*', color="green", label='dmp start')
        ax.plot(dmp_pose[-1, 0], dmp_pose[-1, 1], dmp_pose[-1, 2], '*', color="red", label='dmp end')

        ax.set_box_aspect([1,1,1])
        ax.set_aspect('equal')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.legend()
        plt.show()
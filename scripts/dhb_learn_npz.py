#!/usr/bin/env python3
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import filedialog
from computeDHB import computeDHB
from dhb_ros.reconstructTrajectory import reconstructTrajectory

from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R


from scipy.integrate import cumtrapz

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def load_data(file_name):
    file_path = os.path.join(BASE_DIR, 'data', file_name)
    return np.load(file_path, allow_pickle=True)

def quaternion_to_euler(quaternions):
    eulers = np.zeros((len(quaternions), 3))
    for i in range(len(quaternions)):
        eulers[i] = euler_from_quaternion(quaternions[i])
    return eulers


def train_dhb(pose_traj, time, method):
    if method == 'pos':
        pose0 = pose_traj[0, :3]
        invariants, Hv0, Hw0 = computeDHB(pose_traj[:, :3], pose_traj[:, 3:], method, pose0)
        
        velocities = np.diff(pose_traj[:, :3], axis=0) / np.mean(np.diff(time))
        orient_rates = np.diff(pose_traj[:, 3:], axis=0) / np.mean(np.diff(time))
        return invariants, Hv0, Hw0, pose0, velocities, orient_rates
    elif method == 'vel':
        velocities = np.diff(pose_traj[:, :3], axis=0) / np.mean(np.diff(time))
        orient_rates = np.diff(pose_traj[:, 3:], axis=0) / np.mean(np.diff(time))

        twists = np.zeros((len(time)-1, 6))
        for i in range(len(time)-1):
            Tr = np.array([
                [1, 0, -np.sin(pose_traj[i, 4])],
                [0, np.cos(pose_traj[i, 3]), np.cos(pose_traj[i, 4]) * np.sin(pose_traj[i, 3])],
                [0, -np.sin(pose_traj[i, 3]), np.cos(pose_traj[i, 4]) * np.cos(pose_traj[i, 3])]
            ])
            if orient_rates[i, :].shape[0] == 4:
                # Converti il quaternione in velocità angolari
                delta_q = orient_rates[i, :]
                w = 2 * np.arctan2(np.linalg.norm(delta_q[:3]), delta_q[3])
                axis = delta_q[:3] / np.linalg.norm(delta_q[:3])
                orient_rate = axis * w / np.mean(np.diff(time))
                twists[i, :3] = (Tr @ orient_rate.T).T
            else:
                twists[i, :3] = (Tr @ orient_rates[i, :].T).T

            twists[i, 3:] = velocities[i]


        # Plot linear velocities
        plt.figure(figsize=(10, 6))
        velocity_labels = ['X Velocity', 'Y Velocity', 'Z Velocity']
        for i in range(3):
            plt.subplot(3, 1, i+1)
            plt.plot(time[:-1], velocities[:, i], label=velocity_labels[i])
            plt.ylabel(velocity_labels[i])
            plt.xlabel('Time (s)')
            plt.grid(True)
            plt.legend()
        plt.tight_layout()

        # Plot angular velocities (orient_rates)
        plt.figure(figsize=(10, 6))
        orient_rate_labels = ['Roll Rate', 'Pitch Rate', 'Yaw Rate']
        for i in range(3):
            plt.subplot(3, 1, i+1)
            plt.plot(time[:-1], orient_rates[:, i], label=orient_rate_labels[i])
            plt.ylabel(orient_rate_labels[i])
            plt.xlabel('Time (s)')
            plt.grid(True)
            plt.legend()
        plt.tight_layout()


        invariants, Hv0, Hw0 = computeDHB(twists[:, 3:], twists[:, :3], method)
        return invariants, Hv0, Hw0, velocities, orient_rates, twists
    else:
        raise ValueError(f"Unsupported method: {method}")


def dhb_gen_traj(invariants, Hv0, Hw0, method):
    v, w = reconstructTrajectory(invariants, Hv0, Hw0, method)
    return v, w


if __name__ == '__main__':
    
    # file_name = '/home/catkin_ws/src/nist_cables/processed/clip_closure_pos1.npz'
    # velocity_file = '/home/catkin_ws/src/dhb_ros-General/data/velocities/clip_closure_pos1_ee_velocities.npz'

    # file_name = '/home/catkin_ws/src/nist_cables/processed/clip_closure_pos2.npz'
    # velocity_file = '/home/catkin_ws/src/dhb_ros-General/data/velocities/clip_closure_pos2_ee_velocities.npz'

    # file_name = '/home/catkin_ws/src/nist_cables/processed/push_pos1.npz'
    # velocity_file = '/home/catkin_ws/src/dhb_ros-General/data/velocities/push_pos1_ee_velocities.npz'
    
    # file_name = '/home/catkin_ws/src/nist_cables/processed/locking_pos1_r2.npz'       
    # velocity_file = '/home/catkin_ws/src/dhb_ros-General/data/velocities/locking_pos1_r2_ee_velocities.npz'

    # file_name = '/home/asl_team/catkin_ws/src/nist_cables/processed/T_drop_cable_pos1.npz'
    # velocity_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/T_drop_cable_pos1_ee_velocities.npz'

    file_name = '/home/asl_team/catkin_ws/src/nist_cables/processed/clip_closure_pos3.npz'
    velocity_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/clip_closure_pos3_ee_velocities.npz'

    try:
        file_name
    except NameError:
        root = tk.Tk()
        root.withdraw()  # Hides the root window
        file_name = filedialog.askopenfilename(initialdir=os.path.join(BASE_DIR, "data"), title="Select file")
        root.destroy()  # Closes the Tkinter interface after selection


    plot = True

    # Load data
    data = load_data(file_name)
    pose_traj = data['ee_traj']
    time = data['time'] 


    # Convert quaternions to euler angles (roll, pitch, yaw)
    quaternions = pose_traj[:, 3:]
    euler_angles = quaternion_to_euler(quaternions)

    # Unwrap the angles
    euler_angles_unwrapped = np.unwrap(euler_angles, axis=0)

    print("Quaternions :", quaternions[:5])
    print ("Euler angles :", euler_angles_unwrapped[:5])


    # Load velocity data
    velocity_data = np.load(velocity_file)
    ee_velocities = velocity_data['velocities']
    ee_time_stamps = velocity_data['time_stamps']

    # Adjust the time stamps
    ee_time_stamps = ee_time_stamps - ee_time_stamps[0]

    # Plot the Euler angles
    plt.figure(figsize=(10, 6))
    euler_labels = ['Roll', 'Pitch', 'Yaw']
    for i in range(3):
        plt.subplot(3, 1, i+1)
        plt.plot(ee_time_stamps, euler_angles_unwrapped[:, i], label=euler_labels[i])
        plt.ylabel(euler_labels[i])
        plt.xlabel('Time (s)')
        plt.grid(True)
        plt.legend()
    plt.tight_layout()
    


    # Correct the negative time intervals and calculate the corrected dt
    time = np.sort(time)
    dt = np.diff(time)
    dt = dt[dt > 0]  # Removing the negative temporal intervals
    mean_dt = np.mean(dt)
    
    
    method = 'pos'


    # Linearization
    linear_time = np.linspace(0, time[-1], len(time))

    # Train DHB
    if method == 'pos':
        
        invariants, Hv0, Hw0, pose0, velocities, orient_rates = train_dhb(pose_traj, linear_time, method)

    elif method == 'vel':
        pose_traj = np.hstack((pose_traj[:, :3], euler_angles_unwrapped)) # Use Euler angles for DHB
        print("Pose traj:" , pose_traj[:5])
        invariants, Hv0, Hw0, velocities, orient_rates, twists = train_dhb(pose_traj, linear_time, method)


    # Define save directory and file name
    save_dir = os.path.join(BASE_DIR, 'data', 'dhb')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    base_file_name = f'generated_traj_dhb_{method}'
    save_path = os.path.join(save_dir, base_file_name + '.npz')
    np.savez(save_path, invariants=invariants, Hv0=Hv0, Hw0=Hw0)

    print("DHB training completed and data saved to", save_path)


    time_length = len(invariants)

    # Plot invariant trajectories
    plt.figure(figsize=(15, 6), num='Cartesian velocity to DHB - INVARIANTS')
    dhb_inv_names = ['$m_v$', r'$\theta_v^1$', r'$\theta_v^2$', '$m_\omega$', r'$\theta_\omega^1$', r'$\theta_\omega^2$']
    for i in range(6):
        plt.subplot(2, 3, i+1)
        plt.plot(time[:time_length], invariants[:, i], 'k', linewidth=2)
        plt.ylabel(dhb_inv_names[i])
        plt.grid(True)
    plt.tight_layout()


    # Plot trajectory if required
    if plot:
        v, w = dhb_gen_traj(invariants, Hv0, Hw0, method)

 
        # Components plot 
        fig, axs = plt.subplots(6, 1, figsize=(10, 12))
        traj_labels = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
        if method == 'pos':
            for i, ax in enumerate(axs):
                if i < 3:
                    ax.plot(time[:len(v)], pose_traj[:len(v), i], 'g', label='Original $x_{}$'.format(i+1))
                    ax.plot(time[:len(v)], v[:, i], 'b--', label='Reconstructed $x_{}$'.format(i+1))
                else:
                    ax.plot(time[:len(w)], pose_traj[:len(w), i], 'g', label='Original $\omega_{}$'.format(i-2))
                    ax.plot(time[:len(w)], w[:, i-3], 'b--', label='Reconstructed $\omega_{}$'.format(i-2))
                ax.set_ylabel(traj_labels[i])
                ax.grid(True)
                ax.legend()
        else:
            for i, ax in enumerate(axs):
                if i < 3:
                    ax.plot(time[:len(v)], twists[:len(v), i+3], 'g', label='Original $x_{}$'.format(i+1))
                    ax.plot(time[:len(v)], v[:, i], 'b--', label='Reconstructed $x_{}$'.format(i+1))
                else:
                    ax.plot(time[:len(w)], twists[:len(w), i-3], 'g', label='Original $\omega_{}$'.format(i-2))
                    ax.plot(time[:len(w)], w[:, i-3], 'b--', label='Reconstructed $\omega_{}$'.format(i-2))
                ax.set_ylabel(traj_labels[i])
                ax.grid(True)
                ax.legend()

        plt.tight_layout()

        # Plot ee_velocities 
        fig, axs = plt.subplots(6, 1, figsize=(10, 12))
        velocity_labels = ['X Velocity', 'Y Velocity', 'Z Velocity', 'Roll Rate', 'Pitch Rate', 'Yaw Rate']
        for i in range(3):
            axs[i].plot(ee_time_stamps, ee_velocities[:, i], 'g', label='Original Velocity $v_{}$'.format(i+1))
            # axs[i].plot(time[:len(v)], v[:, i], 'b--', label='Reconstructed Velocity $v_{}$'.format(i+1))
            axs[i].set_ylabel(velocity_labels[i])
            axs[i].grid(True)
            axs[i].legend()

        for i in range(3):
            axs[i+3].plot(ee_time_stamps, ee_velocities[:, i+3], 'r', label='Original $\omega_{}$'.format(i+1))
            # axs[i+3].plot(time[:len(w)], w[:, i], 'm--', label='Reconstructed $\omega_{}$'.format(i+1))
            axs[i+3].set_ylabel(velocity_labels[i+3])
            axs[i+3].grid(True)
            axs[i+3].legend()

        plt.tight_layout()


        # # Plot delle componenti delle velocità lineari e angolari originali e ricostruite nello stesso plot
        # fig, axs = plt.subplots(6, 1, figsize=(10, 18))
        # velocity_labels = ['X Velocity', 'Y Velocity', 'Z Velocity']
        # orientation_labels = ['Roll Rate', 'Pitch Rate', 'Yaw Rate']
        # for i in range(3):
        #     axs[i].plot(time[:len(velocities)], velocities[:len(velocities), i], 'g', label='Original Velocity $v_{}$'.format(i+1))
        #     axs[i].plot(time[:len(v)], v[:, i], 'b--', label='Reconstructed Velocity $v_{}$'.format(i+1))
        #     axs[i].set_ylabel(velocity_labels[i])
        #     axs[i].grid(True)
        #     axs[i].legend()

        # for i in range(3):
        #     axs[i+3].plot(time[:len(orient_rates)], twists[:len(orient_rates), i], 'r', label='Original $\omega_{}$'.format(i+1))
        #     axs[i+3].plot(time[:len(w)], w[:, i], 'm--', label='Reconstructed $\omega_{}$'.format(i+1))
        #     axs[i+3].set_ylabel(orientation_labels[i])
        #     axs[i+3].grid(True)
        #     axs[i+3].legend()

        # plt.tight_layout()


        # plt.show()

        # # 3D PLOT - ORIGINAL TRAJECTORY
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.plot(pose_traj[:, 0], pose_traj[:, 1], pose_traj[:, 2], 'r-', label='Original Trajectory')
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')
        # ax.legend()


        # # Plot original and reconstructed velocity
        # plt.figure(figsize=(10, 6), num='DHB to Cartesian velocity')
        # for i in range(6):
        #     plt.subplot(2, 3, i+1)
        #     if method == 'pos':
        #         if i < 3:
        #             plt.plot(time[:len(w)], pose_traj[:len(w), i+3], 'g', linewidth=5, label='Original $\omega_{}$'.format(i+1))
        #             plt.plot(time[:len(w)], w[:, i], 'b', linewidth=2, label='Reconstructed $\omega_{}$'.format(i+1))
        #             plt.ylabel('$\omega_{}$'.format(i+1))
        #         else:
        #             plt.plot(time[:len(v)], pose_traj[:len(v), i-3], 'g', linewidth=5, label='Original $x_{}$'.format(i-2))
        #             plt.plot(time[:len(v)], v[:, i-3], 'b', linewidth=2, label='Reconstructed $x_{}$'.format(i-2))
        #             plt.ylabel('$x_{}$'.format(i-2))
        #     else:
        #         if i < 3:
        #             plt.plot(time[:len(w)], twists[:len(w), i], 'g', linewidth=5, label='Original $\omega_{}$'.format(i+1))
        #             plt.plot(time[:len(w)], w[:, i], 'b', linewidth=2, label='Reconstructed $\omega_{}$'.format(i+1))
        #             plt.ylabel('$\omega_{}$'.format(i+1))
        #         else:
        #             plt.plot(time[:len(v)], twists[:len(v), i], 'g', linewidth=5, label='Original $x_{}$'.format(i-2))
        #             plt.plot(time[:len(v)], v[:, i-3], 'b', linewidth=2, label='Reconstructed $x_{}$'.format(i-2))
        #             plt.ylabel('$x_{}$'.format(i-2))
        #     plt.grid(True)
        #     plt.legend()
        # plt.tight_layout()



        # # 3D PLOT
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.plot(twists[:, 3], twists[:, 4], twists[:, 5], 'r-', label='Original Linear Velocities')
        # ax.plot(v[:, 0], v[:, 1], v[:, 2], 'b--', label='DHB Reconstructed Linear Velocities')
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')
        # ax.legend()


        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.plot(twists[:, 0], twists[:, 1], twists[:, 2], 'r-', label='Original Angular Velocities')
        # ax.plot(w[:, 0], w[:, 1], w[:, 2], 'b--', label='DHB Reconstructed Angular Velocities')
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')
        # ax.legend()
        plt.show()

        # Calculate the number of valid samples
        n_valid = min(len(v), len(w), len(twists))

        velocity_errors = velocities[:n_valid] - v[:n_valid]
        squared_velocity_errors = np.square(velocity_errors)
        sum_squared_velocity_errors = np.sum(squared_velocity_errors, axis=0)
        rmse_velocity = np.sqrt(sum_squared_velocity_errors / n_valid)

        orientation_errors = twists[:n_valid, :3] - w[:n_valid]
        squared_orientation_errors = np.square(orientation_errors)
        sum_squared_orientation_errors = np.sum(squared_orientation_errors, axis=0)
        rmse_orientation = np.sqrt(sum_squared_orientation_errors / n_valid)

        print("Squared Velocity Errors (First 5):", squared_velocity_errors[:5])
        print("Sum of Squared Velocity Errors:", sum_squared_velocity_errors)
        print("RMSE for Velocities:", rmse_velocity)

        print("Squared Orientation Errors (First 5):", squared_orientation_errors[:5])
        print("Sum of Squared Orientation Errors:", sum_squared_orientation_errors)
        print("RMSE for Orientations:", rmse_orientation)

        plt.tight_layout()
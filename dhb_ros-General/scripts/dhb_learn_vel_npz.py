import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import filedialog
from computeDHB import computeDHB
from pytransform3d.rotations import quaternion_integrate
from dhb_ros.reconstructTrajectory import reconstructTrajectory

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def load_data(file_name):
    file_path = os.path.join(BASE_DIR, 'data', file_name)
    return np.load(file_path, allow_pickle=True)

def train_dhb(velocities, orient_rates, time, method):
    if method == 'vel':
        twists = np.hstack((orient_rates, velocities))
        invariants, Hv0, Hw0 = computeDHB(velocities, orient_rates, method)

        # # Plot linear velocities (velocities)
        # plt.figure(figsize=(10, 6))
        # velocity_labels = ['Linear Velocity X', 'Linear Velocity Y', 'Linear Velocity Z']
        # for i in range(3):
        #     plt.subplot(3, 1, i+1)
        #     plt.plot(time, velocities[:, i], label=velocity_labels[i])
        #     plt.ylabel(velocity_labels[i])
        #     plt.xlabel('Time (s)')
        #     plt.grid(True)
        #     plt.legend()
        # plt.tight_layout()

        # # Plot angular velocities (orient_rates)
        # plt.figure(figsize=(10, 6))
        # orient_rate_labels = ['Angular Velocity Roll', 'Angular Velocity Pitch', 'Angular Velocity Yaw']
        # for i in range(3):
        #     plt.subplot(3, 1, i+1)
        #     plt.plot(time, orient_rates[:, i], label=orient_rate_labels[i])
        #     plt.ylabel(orient_rate_labels[i])
        #     plt.xlabel('Time (s)')
        #     plt.grid(True)
        #     plt.legend()
        # plt.tight_layout()

        return invariants, Hv0, Hw0, velocities, orient_rates, twists
    else:
        raise ValueError(f"Unsupported method: {method}")

def dhb_gen_traj(invariants, Hv0, Hw0, method):
    v, w = reconstructTrajectory(invariants, Hv0, Hw0, method)
    return v, w

def integrate_trajectory(v, w, dt, initial_position, initial_orientation):
    num_points = v.shape[0]
    positions = np.zeros((num_points, 3))
    positions[0] = initial_position
    

    # Integrate angular velocities to obtain quaternions
    quaternions = quaternion_integrate(w, q0=initial_orientation, dt=dt)
    orientations = np.array([quaternion for quaternion in quaternions])

    for i in range(1, num_points):
        positions[i] = positions[i-1] + v[i] * dt

    return positions, orientations

if __name__ == '__main__':
    # velocity_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/starting_point_3_ee_velocities.npz' # CHANGE HERE 
    # velocity_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/approach_cable_3_ee_velocities.npz' # CHANGE HERE 
    # velocity_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/put_in_clip_3_ee_velocities.npz' # CHANGE HERE 
    # velocity_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/approach_clip_3_ee_velocities.npz' # CHANGE HERE 
    # velocity_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/close_clip_3_ee_velocities.npz' # CHANGE HERE 
    velocity_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/lock_clip_3_ee_velocities.npz' # CHANGE HERE 
    

    try:
        velocity_file
    except NameError:
        root = tk.Tk()
        root.withdraw()  # Hides the root window
        velocity_file = filedialog.askopenfilename(initialdir=os.path.join(BASE_DIR, "data"), title="Select file")
        root.destroy()  # Closes the Tkinter interface after selection

    plot = True

    # Load velocity data
    velocity_data = load_data(velocity_file)
    # import pdb
    # pdb.set_trace()
    ee_velocities = velocity_data['velocities']
    ee_time_stamps = velocity_data['time_stamps']
    poses = velocity_data['poses']
    initial_position = poses[0, :3]
    initial_orientation = poses[0, 3:]

    # Adjust the time stamps
    ee_time_stamps = ee_time_stamps - ee_time_stamps[0]

    # Duration
    duration = ee_time_stamps[-1] - ee_time_stamps[0]

    # Separate linear and angular velocities
    linear_velocities = ee_velocities[:, :3]
    angular_velocities = ee_velocities[:, 3:]

    plt.figure()
    plt.plot(ee_time_stamps, linear_velocities)
    plt.title("linear")
    plt.figure()
    plt.plot(ee_time_stamps, angular_velocities)
    plt.title("angular")
    plt.show()

    method = 'vel'

    # Train DHB
    invariants, Hv0, Hw0, _, _, twists = train_dhb(linear_velocities, angular_velocities, ee_time_stamps, method)
    import pdb
    pdb.set_trace()

    # Define save directory and file name
    save_dir = os.path.join(BASE_DIR, 'data', 'dhb')
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    traj_file_name = os.path.basename(velocity_file).split('.')[0]
    base_file_name = f'generated_traj_dhb_{method}_{traj_file_name}'
    save_path = os.path.join(save_dir, base_file_name + '.npz')
    print(save_path)

    # Generate trajectory from DHB invariants
    v, w = dhb_gen_traj(invariants, Hv0, Hw0, method)
    print(v.shape, w.shape)

    # Integrate trajectory
    dt = ee_time_stamps[1] - ee_time_stamps[0]  # Assume timestamps are equidistant
    positions, orientations = integrate_trajectory(v, w, dt, initial_position, initial_orientation)
    print(positions.shape, orientations.shape)

    # Integrate traj. from original vel 
    positions_original, orientations_original = integrate_trajectory(linear_velocities, angular_velocities, dt, initial_position, initial_orientation)
    print(positions_original.shape, orientations_original.shape)

    # Trim time stamps and positions to the same length
    min_length = min(len(ee_time_stamps), len(positions))
    # ee_time_stamps = ee_time_stamps[:min_length]
    # positions = positions[:min_length]
    # positions_original = positions_original[:min_length]
    # orientations_original = orientations_original[:min_length]

    # Save invariants and trajectory data
    np.savez(save_path, invariants=invariants, Hv0=Hv0, Hw0=Hw0, duration=duration,
             positions=positions, orientations=orientations)

    print("DHB training completed and data saved to", save_path)

    time_length = len(invariants)



    # Calculate the number of valid samples
    n_valid = min(len(v), len(w), len(twists))

    velocity_errors = linear_velocities[:n_valid] - v[:n_valid]
    squared_velocity_errors = np.square(velocity_errors)
    sum_squared_velocity_errors = np.sum(squared_velocity_errors, axis=0)
    rmse_velocity = np.sqrt(sum_squared_velocity_errors / n_valid)

    orientation_errors = angular_velocities[:n_valid] - w[:n_valid]
    squared_orientation_errors = np.square(orientation_errors)
    sum_squared_orientation_errors = np.sum(squared_orientation_errors, axis=0)
    rmse_orientation = np.sqrt(sum_squared_orientation_errors / n_valid)

    print("Squared Velocity Errors (First 5):", squared_velocity_errors[:5])
    print("Sum of Squared Velocity Errors:", sum_squared_velocity_errors)
    print("RMSE for Velocities:", rmse_velocity)

    print("Squared Orientation Errors (First 5):", squared_orientation_errors[:5])
    print("Sum of Squared Orientation Errors:", sum_squared_orientation_errors)
    print("RMSE for Orientations:", rmse_orientation)

    # plt.show()
#!/usr/bin/env python3
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import filedialog
from computeDHB import computeDHB
from reconstructTrajectory import reconstructTrajectory

from scipy.integrate import cumtrapz

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def load_data(file_name):
    file_path = os.path.join(BASE_DIR, 'data', file_name)
    return np.load(file_path, allow_pickle=True)


def train_dhb(pose_traj, time, method):
    if method == 'pos':
        # Calcolo degli invarianti DHB in modalità "pos"
        pose0 = pose_traj[0, :3]
        invariants, Hv0, Hw0 = computeDHB(pose_traj[:, :3], pose_traj[:, 3:], method, pose0)

        velocities = np.diff(np.vstack((x, y, z)).T, axis=0) / dt
        orient_rates = np.diff(np.vstack((roll, pitch, yaw)).T, axis=0) / dt


        return invariants, Hv0, Hw0, pose0, velocities, orient_rates
    elif method == 'vel':
        # Calcolo delle velocità lineari e angolari
        velocities = np.diff(np.vstack((x, y, z)).T, axis=0) / dt
        orient_rates = np.diff(np.vstack((roll, pitch, yaw)).T, axis=0) / dt


        # Adjustment for orientation rates
        twists = np.zeros((N-1, 6))
        for i in range(N-1):
            Tr = np.array([
                [1, 0, -np.sin(pitch[i])],
                [0, np.cos(roll[i]), np.cos(pitch[i]) * np.sin(roll[i])],
                [0, -np.sin(roll[i]), np.cos(pitch[i]) * np.cos(roll[i])]
            ])
            twists[i, :3] = (Tr @ orient_rates[i, :].T).T
            twists[i, 3:] = velocities[i]

        
        # Calcolo degli invarianti DHB in modalità "vel"
        invariants, Hv0, Hw0 = computeDHB(twists[:, 3:], twists[:, :3], method)

        return invariants, Hv0, Hw0, velocities, orient_rates, twists
    else:
        raise ValueError(f"Unsupported method: {method}")


def dhb_gen_traj(invariants, Hv0, Hw0, method):
    v, w = reconstructTrajectory(invariants, Hv0, Hw0, method)

    return v, w


if __name__ == '__main__':

    # file_name = '/home/catkin_ws/src/nist_cables/processed/clip_closure_pos1_record_1.npz'
    # try:
    #     file_name
    # except NameError:
    #     root = tk.Tk()
    #     root.withdraw()  # Hides the root window
    #     file_name = filedialog.askopenfilename(initialdir=os.path.join(BASE_DIR, "data"), title="Select file")
    #     root.destroy()  # Closes the Tkinter interface after selection

    # # Load data
    # data = load_data(file_name)
    # pose_traj = data['ee_traj']
    # time = data['time'] 

    # # Correct the negative time intervals and calculate the corrected dt
    # time = np.sort(time)
    # dt = np.diff(time)
    # dt = dt[dt > 0]  # Removing the negative temporal intervals
    # mean_dt = np.mean(dt)
    
    # Utilizza la traiettoria definita nel mainDHB
    dt = 0.1  # Tempo di campionamento
    tf = 4    # Tempo totale
    N = 1000  # Numero di campioni 

    # Vettore temporale
    T = np.linspace(0, tf, N)

    # Tracciato 6DOF
    x = 0.1 * np.exp(T)
    y = 5 + 1.5 * np.sin(T)
    z = np.cos(T)

    roll = 0.5 * np.sin(T)
    pitch = np.cos(T)
    yaw = 0.1 * T

    pose_traj = np.vstack((x, y, z, roll, pitch, yaw)).T
    time = T

    plot = True
    method = 'vel'


    # Linearization
    linear_time = np.linspace(0, time[-1], len(time))

    # Train DHB
    if method == 'pos':
        
        invariants, Hv0, Hw0, pose0, velocities, orient_rates = train_dhb(pose_traj, linear_time, method)

    elif  method == 'vel':
        
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
    plt.figure(figsize=(10, 6), num='Cartesian velocity to DHB')
    dhb_inv_names = ['$m_v$', r'$\theta_v^1$', r'$\theta_v^2$', '$m_\omega$', r'$\theta_\omega^1$', r'$\theta_\omega^2$']
    for i in range(6):
        plt.subplot(2, 3, i+1)
        plt.plot(T[:time_length], invariants[:, i], 'k', linewidth=2)
        plt.ylabel(dhb_inv_names[i])
        plt.grid(True)
    plt.tight_layout()


    # Plot trajectory if required
    if plot:
        start = pose_traj[0, :]
        goal = pose_traj[-1, :]
        v, w = dhb_gen_traj(invariants, Hv0, Hw0, method)

        if method == 'pos': 


            # ERRORS
            n_valid = min(v.shape[0], twists.shape[0])

            # Stampa le differenze dirette
            direct_diff_vr = v[:n_valid] - twists[:n_valid, 3:6]
            direct_diff_wr = w[:n_valid] - twists[:n_valid, 0:3]

            # Calcolo degli errori di ricostruzione
            errSP = errSP = np.hstack([(direct_diff_vr)**2, (direct_diff_wr)**2])


            sum_squared_errors_1 = np.sum(errSP[:, :3])
            sum_squared_errors_2 = np.sum(errSP[:, 3:6])   
            
            # Calcola RMSE per ciascun gruppo
            RMSE_1 = np.sqrt(sum_squared_errors_1 / (N-3))
            RMSE_2 = np.sqrt(sum_squared_errors_2 / (N-3))

            print("RMSE 1:", RMSE_1)
            print("RMSE 2:", RMSE_2)

            # PLOT
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(twists[:,3], twists[:, 4], twists[:, 5], 'r-', label='Original Linear Velocities')
            ax.plot(v[:, 0], v[:, 1], v[:, 2], 'b--', label='DHB Reconstructed Linear Velocities')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.legend()

            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(twists[:,0], twists[:, 1], twists[:, 2], 'r-', label='Original Angular Velocities')
            ax.plot(w[:, 0], w[:, 1], w[:, 2], 'b--', label='DHB Reconstructed Angular Velocities')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.legend()
            plt.show()


        elif method == 'vel':

            # ERRORS
            n_valid = min(v.shape[0], twists.shape[0])

            # Stampa le differenze dirette
            direct_diff_vr = v[:n_valid] - twists[:n_valid, 3:6]
            direct_diff_wr = w[:n_valid] - twists[:n_valid, 0:3]

            # Calcolo degli errori di ricostruzione
            errSP = errSP = np.hstack([(direct_diff_vr)**2, (direct_diff_wr)**2])


            sum_squared_errors_1 = np.sum(errSP[:, :3])
            sum_squared_errors_2 = np.sum(errSP[:, 3:6])   
            
            # Calcola RMSE per ciascun gruppo
            RMSE_1 = np.sqrt(sum_squared_errors_1 / (N-3))
            RMSE_2 = np.sqrt(sum_squared_errors_2 / (N-3))

            print("RMSE 1:", RMSE_1)
            print("RMSE 2:", RMSE_2)

            # PLOT
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(twists[:,3], twists[:, 4], twists[:, 5], 'r-', label='Original Linear Velocities')
            ax.plot(v[:, 0], v[:, 1], v[:, 2], 'b--', label='DHB Reconstructed Linear Velocities')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.legend()

            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(twists[:,0], twists[:, 1], twists[:, 2], 'r-', label='Original Angular Velocities')
            ax.plot(w[:, 0], w[:, 1], w[:, 2], 'b--', label='DHB Reconstructed Angular Velocities')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.legend()
            plt.show()

   
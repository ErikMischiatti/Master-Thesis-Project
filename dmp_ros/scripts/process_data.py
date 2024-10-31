#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
from bagpy import bagreader
import pandas as pd
import pytransform3d.rotations as pr
import pytransform3d.transformations as pt
import os
import re
import tkinter as tk
from tkinter import filedialog

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def flip_quaternions(quat, ref_quat=[1, 0, 0, 0]):
    return -quat if np.dot(ref_quat, quat) < 0 else quat

def flip_quaternion_trajectory(quat_traj):
    transposed = quat_traj.shape[0] != 4
    if transposed:
        quat_traj = quat_traj.T  # Transpose to (4, n) for consistency
    
    flipped_quat_traj = np.empty_like(quat_traj)
    flipped_quat_traj[:, 0] = quat_traj[:, 0]
    for i in range(1, quat_traj.shape[1]):
        flipped_quat_traj[:, i] = flip_quaternions(quat_traj[:, i], flipped_quat_traj[:, i - 1])

    return flipped_quat_traj.T if transposed else flipped_quat_traj

def load_data(file_name):
    print(f"loading bag file: {file_name}")
    full_file_path = os.path.join(BASE_DIR, "data", file_name)
    bag = bagreader(full_file_path)
    
    pose_df = pd.read_csv(bag.message_by_topic('/franka_state_controller/O_T_EE'))
    time = np.array(pose_df['Time']) - pose_df['Time'][0]
    pose = pose_df[['pose.position.x', 'pose.position.y', 'pose.position.z', 
                            'pose.orientation.w', 'pose.orientation.x', 'pose.orientation.y', 'pose.orientation.z']].values
    
    pose[:,3:] = flip_quaternion_trajectory(pose[:,3:])

    data = {'pose': pose, 'time': time}

    return data

def segment_data(data):
    # Create subplots
    fig = plt.figure()

   # Creating subplots on the left column
    ax1 = fig.add_subplot(7, 2, 1)  
    ax2 = fig.add_subplot(7, 2, 3)  
    ax3 = fig.add_subplot(7, 2, 5)
    
    ax4 = fig.add_subplot(7, 2, 7)  
    ax5 = fig.add_subplot(7, 2, 9)  
    ax6 = fig.add_subplot(7, 2, 11) 
    ax7 = fig.add_subplot(7, 2, 13) 

    # Creating one large subplot on the right column
    ax8 = fig.add_subplot(1, 2, 2, projection='3d')  # The entire right column

    # Plotting the trajectories
    ax1.plot(data["time"], data["pose"][:,0], picker=5)
    ax1.set_ylabel('X')
    ax2.plot(data["time"], data["pose"][:,1], picker=5)
    ax2.set_ylabel('Y')
    ax3.plot(data["time"], data["pose"][:,2], picker=5)
    ax3.set_ylabel('Z')
    ax4.plot(data["time"], data["pose"][:,3], picker=5)
    ax4.set_ylabel('q_W')
    ax5.plot(data["time"], data["pose"][:,4], picker=5)
    ax5.set_ylabel('q_x')
    ax6.plot(data["time"], data["pose"][:,5], picker=5)
    ax6.set_ylabel('q_y')
    ax7.plot(data["time"], data["pose"][:,6], picker=5)
    ax7.set_ylabel('q_z')
    ax8.plot(data["pose"][:,0], data["pose"][:,1], data["pose"][:,2], picker=5)
    ax8.set_xlabel('X')
    ax8.set_ylabel('Y')
    ax8.set_zlabel('Z')

    ax8.set_box_aspect([1,1,1])
    ax8.set_aspect('equal')

    selected_indices = []

    def onpick(event):
        if hasattr(event.artist, 'get_xdata') and hasattr(event.artist, 'get_ydata'):
            ind = event.ind[0]  # Take the first index
            time_val = event.artist.get_xdata()[ind]

            # Find the nearest index in the time array
            index = np.abs(data["time"] - time_val).argmin()

            # Add markers to all subplots
            x_val, y_val, z_val = data["pose"][index, 0], data["pose"][index, 1], data["pose"][index, 2]
            qw_val, qx_val, qy_val, qz_val = data["pose"][index, 3], data["pose"][index, 4], data["pose"][index, 5], data["pose"][index, 6]
            ax1.plot(data["time"][index], x_val, 'ro')
            ax2.plot(data["time"][index], y_val, 'ro')
            ax3.plot(data["time"][index], z_val, 'ro')
            ax4.plot(data["time"][index], qw_val, 'ro')
            ax5.plot(data["time"][index], qx_val, 'ro')
            ax6.plot(data["time"][index], qy_val, 'ro')
            ax7.plot(data["time"][index], qz_val, 'ro')
            ax8.plot([x_val], [y_val], [z_val], 'ro')

            selected_indices.append(index)  # Store the index of the selected point

            fig.canvas.draw()  # Update the figure

    fig.canvas.mpl_connect('pick_event', onpick)
    plt.show()

    # Print selected indices
    print("Selected indices:")
    print(selected_indices)

    return selected_indices

def save_segments(data, segment_indices, original_file):

    if not segment_indices:
        print("No segments selected. No data will be saved.")
        return  # Early exit if no segments are selected
    
    # Ensure the save directory exists
    full_save_dir = os.path.join(BASE_DIR, 'data', 'segments')
    if not os.path.exists(full_save_dir):
        os.makedirs(full_save_dir)
    
    base_name = os.path.splitext(os.path.basename(original_file))[0]

    # Extend segment_indices to include the beginning and end of the data
    full_indices = [0] + list(segment_indices) + [len(data[list(data.keys())[0]])]  # Assuming all data arrays are of the same length

    # Save each segment into a separate file
    for i in range(len(full_indices) - 1):
        start_index = full_indices[i]
        end_index = full_indices[i + 1]
        data_ = {key: value[start_index:end_index] for key, value in data.items()}
        data_['time'] = data_['time'] - data_['time'][0]
        np.save(os.path.join(full_save_dir, f"{base_name}_segment_{i}.npy"), data_)

if __name__ == '__main__':
    file = "new/_2024-09-02-11-54-48.bag"  # Default file path; comment to select file from UI
    try:
        file
    except NameError:
        root = tk.Tk()
        root.withdraw()  # Hides the root window
        file = filedialog.askopenfilename(initialdir=os.path.join(BASE_DIR, "data"), title="Select file")
        root.destroy()  # Closes the Tkinter interface after selection

    if file:
        data = load_data(file_name=file)
        segment_indices = segment_data(data)

        if segment_indices:  # Check if any segments are selected
            save_segments(data=data, segment_indices=segment_indices, original_file=file)
        else:
            print("No segments selected. Exiting without saving.")

    else:
        print("No file selected")
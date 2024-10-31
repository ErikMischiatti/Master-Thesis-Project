import numpy as np

file_path = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/dhb/generated_traj_dhb_vel_push_pos1_ee_velocities.npz'  # Choose the .npz file to check
data = np.load(file_path)
print(data.files)  
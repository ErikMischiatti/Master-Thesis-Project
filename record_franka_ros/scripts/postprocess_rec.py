import numpy as np
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
from computeDHB import computeDHB
from dhb_ros.reconstructTrajectory import reconstructTrajectory
from pytransform3d.rotations import quaternion_integrate

def moving_average(a, n=3):
    ret = np.cumsum(a, dtype=float, axis=0)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


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

bag_file = "/home/asl_team/catkin_ws/src/record_franka_ros/data/data_raw/ours_new/recording_2024-10-30-10-01-02.bag"
# with rosbag.Bag(bag_file, 'r') as bag:
#         for topic, msg, t in bag.read_messages(topics=['/franka_state_controller/O_T_EE_vel']):
#             import pdb
#             pdb.set_trace()


## CHECK VALUES !
# start_point = 0
# cut_point = -1

start_point = 0
cut_point = -1
w_size = 10

b = bagreader(bag_file)

data = b.message_by_topic('/franka_state_controller/O_T_EE')
data_pd = pd.read_csv(data)

initial_pose = np.array([data_pd["pose.position.x"].values,
                         data_pd["pose.position.y"].values,
                         data_pd["pose.position.z"].values,
                         data_pd["pose.orientation.w"].values,
                         data_pd["pose.orientation.x"].values,
                         data_pd["pose.orientation.y"].values,
                         data_pd["pose.orientation.z"].values]).T

initial_pose = initial_pose[start_point:cut_point]

# import pdb
# pdb.set_trace()

data = b.message_by_topic('/franka_state_controller/O_T_EE_vel')
data_pd = pd.read_csv(data)

time = data_pd["Time"].values
if start_point!=0:
    time -= time[start_point-w_size+1]
else:
    time -= time[start_point]

# import pdb
# pdb.set_trace()

linear = np.stack([data_pd["twist.linear.x"], data_pd["twist.linear.y"], data_pd["twist.linear.z"]]).T
angular = np.stack([data_pd["twist.angular.x"], data_pd["twist.angular.y"], data_pd["twist.angular.z"]]).T


linear_f = moving_average(linear, n=w_size)[start_point:cut_point]
angular_f = moving_average(angular, n=w_size)[start_point:cut_point]
if start_point!=0:
    time_f = time[start_point-w_size+1:cut_point-w_size+1]
else:
    time_f = time[start_point:cut_point-w_size+1]

# import pdb
# pdb.set_trace()

# plt.plot(time, linear, label="raw")
# plt.plot(time_f, linear_f, label="filtered")
plt.plot(linear_f, label="filtered")
plt.legend()
plt.figure()
# plt.plot(time, angular, label="raw")
plt.plot(angular_f, label="filtered")
# plt.plot(angular_f, label="filtered")
plt.legend()
plt.show()

invariants, Hv0, Hw0, _, _, twists = train_dhb(linear_f, angular_f, time_f, "vel")

fig, axs = plt.subplots(6, 1)
axs[0].plot(invariants[:, 0])
axs[1].plot(invariants[:, 1])
axs[2].plot(invariants[:, 2])
axs[3].plot(invariants[:, 3])
axs[4].plot(invariants[:, 4])
axs[5].plot(invariants[:, 5])
plt.show()

v, w = dhb_gen_traj(invariants, Hv0, Hw0, "vel")

plt.plot(linear_f, label="base")
plt.plot(v, label="recon")
plt.legend()
plt.figure()
plt.plot(angular_f, label="base")
plt.plot(w, label="recon")
plt.legend()
plt.show()

# import pdb
# pdb.set_trace()

dt = time_f[-2]/len(w)
integrated = integrate_trajectory(v, w, dt, initial_pose[0, :3], initial_pose[0, 3:])
# np.savez(output_file, poses=poses, velocities=velocities, forces=forces, moments=moments, time_stamps=time_stamps)



plt.plot(initial_pose[:-2], label="raw")
plt.plot(integrated[0], label="int")
plt.plot(integrated[1], label="int")
plt.legend()
plt.show()

save_path = "./ours_alignment"
np.savez(save_path, invariants=invariants, Hv0=Hv0, Hw0=Hw0, duration=time_f[-2],
             positions=integrated[0], orientations=integrated[1], linear = v, angular = w)

# import pdb
# pdb.set_trace()
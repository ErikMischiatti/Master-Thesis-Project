import numpy as np
import rosbag
from franka_msgs.msg import FrankaState
from tf.transformations import quaternion_from_matrix, translation_from_matrix

def correct_quaternion_traj(quat_traj):
    correct_quat_traj = quat_traj.copy()
    for i in range(len(correct_quat_traj) - 1):
        if np.dot(correct_quat_traj[i], correct_quat_traj[i + 1]) < 0:
            correct_quat_traj[i + 1] = -correct_quat_traj[i + 1]
    return correct_quat_traj

def get_tf_mat(i, dh):
    a, d, alpha, theta = dh[i]
    q = theta
    return np.array([
        [np.cos(q), -np.sin(q), 0, a],
        [np.sin(q) * np.cos(alpha), np.cos(q) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
        [np.sin(q) * np.sin(alpha), np.cos(q) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
        [0, 0, 0, 1]
    ])

def get_jacobian(joint_angles):
    # Define your Denavit-Hartenberg parameters here, specific to Franka Emika Panda
    dh_params = np.array([
        [0, 0.333, 0, joint_angles[0]],
        [0, 0, -np.pi / 2, joint_angles[1]],
        [0, 0.316, np.pi / 2, joint_angles[2]],
        [0.0825, 0, np.pi / 2, joint_angles[3]],
        [-0.0825, 0.384, -np.pi / 2, joint_angles[4]],
        [0, 0, np.pi / 2, joint_angles[5]],
        [0.088, 0, np.pi / 2, joint_angles[6]],
        [0, 0.107, 0, 0],
        [0, 0, 0, -np.pi / 4],
        [0.0, 0.1444, 0, 0]
    ], dtype=np.float64)

    T_EE = np.identity(4)
    for i in range(7 + 3):  # Total joints + fixed frames
        T_EE = T_EE @ get_tf_mat(i, dh_params)

    J = np.zeros((6, 10))
    T = np.identity(4)
    for i in range(7 + 3):  # Total joints + fixed frames
        T = T @ get_tf_mat(i, dh_params)
        p = T_EE[:3, 3] - T[:3, 3]
        z = T[:3, 2]
        J[:3, i] = np.cross(z, p)
        J[3:, i] = z

    return J[:, :7]

def process_bag_file(bag_file, output_file):
    poses = []
    forces = []
    moments = []
    velocities = []
    time_stamps = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/franka_state_controller/franka_states']):
            trans_mtx = np.array(msg.O_T_EE).reshape(4, 4).T

            rot = quaternion_from_matrix(trans_mtx)
            rot = [rot[3], rot[0], rot[1], rot[2]]  # Reorder to (w,x,y,z)
            trans = translation_from_matrix(trans_mtx)

            pose = [*trans, *rot]
            joint_angles = np.array(msg.q)
            joint_velocities = np.array(msg.dq)
            joint_velocity_vector = get_jacobian(joint_angles) @ joint_velocities[..., np.newaxis]  # Calculate EE velocity in cartesian

            force = msg.O_F_ext_hat_K[:3]  # First three are force data
            moment = msg.O_F_ext_hat_K[3:]  # Last three are moment data

            forces.append(force)
            moments.append(moment)
            velocities.append(joint_velocity_vector.flatten())
            time_stamps.append(t.to_sec())
            poses.append(pose)

    poses = np.array(poses)
    forces = np.array(forces)
    moments = np.array(moments)
    velocities = np.array(velocities)
    time_stamps = np.array(time_stamps)
    time_stamps -= time_stamps[0]  # Normalize to start at zero

    np.savez(output_file, poses=poses, velocities=velocities, forces=forces, moments=moments, time_stamps=time_stamps)
    print(f"Data saved to {output_file}")

if __name__ == '__main__':
    # bag_file = '/home/asl_team/catkin_ws/src/record_franka_ros/data/data_raw/starting_point_3.bag'
    # output_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/starting_point_3_ee_velocities.npz'

    # bag_file = '/home/asl_team/catkin_ws/src/record_franka_ros/data/data_raw/approach_cable_3.bag'
    # output_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/approach_cable_3_ee_velocities.npz'

    # bag_file = '/home/asl_team/catkin_ws/src/record_franka_ros/data/data_raw/put_in_clip_3.bag'
    # output_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/put_in_clip_3_ee_velocities.npz'

    # bag_file = '/home/asl_team/catkin_ws/src/record_franka_ros/data/data_raw/approach_clip_3.bag'
    # output_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/approach_clip_3_ee_velocities.npz'

    # bag_file = '/home/asl_team/catkin_ws/src/record_franka_ros/data/data_raw/close_clip_3.bag'
    # output_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/close_clip_3_ee_velocities.npz'

    bag_file = '/home/asl_team/catkin_ws/src/record_franka_ros/data/data_raw/lock_clip_3.bag'
    output_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/lock_clip_3_ee_velocities.npz'
    process_bag_file(bag_file, output_file)

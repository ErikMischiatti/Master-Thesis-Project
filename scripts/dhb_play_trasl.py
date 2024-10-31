#!/usr/bin/env python3

# IMPLEMENTATION OF THE trajectory_generalization FUNCTION
    # Define the function trajectory_generalization(position_traj, velocity_traj, start, goal) that generalizes a given trajectory.

# IMPLEMENTATION OF /cable_pose detection 

import rospy
import rospkg
import numpy as np
import matplotlib.pyplot as plt
import actionlib
import cartesian_control_msgs.msg

from scipy.signal import butter, filtfilt
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped
from scipy.spatial.transform import Rotation as R
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from mpl_toolkits.mplot3d import Axes3D


class DHB_ROS:
    def __init__(self):
        ############################################## ROS setup
        rospy.init_node("dhb_ros")

        self.motion = rospy.get_param('~motion')
        self.pose_cable = None  # Memorizza la posizione del cavo
        
        # Subscriber per la posizione del cavo
        self.sub_cable_pose = rospy.Subscriber("/cable_pose", PoseStamped, self.cable_pose_cb)

        self.sub_robot_state = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.state_cb, queue_size=10)
        self.sub_F_ext_base = rospy.Subscriber("/franka_state_controller/F_ext_base", WrenchStamped, self.wrench_cb, queue_size=10)
        self.sub_gripper = rospy.Subscriber("/franka_gripper/joint_states", JointState, self.gripper_cb, queue_size=1)

        self.pub_des_pose = rospy.Publisher("/trajectory_cartesian_impedance_controller/equilibrium_pose", PoseStamped, queue_size=1)
        self.pub_alpha = rospy.Publisher("/alpha", Float64, queue_size=1)

        self.ros_rate = 750
        self.rate = rospy.Rate(self.ros_rate)

        self.client = actionlib.SimpleActionClient('/trajectory_cartesian_impedance_controller/follow_cartesian_trajectory', cartesian_control_msgs.msg.FollowCartesianTrajectoryAction)

        rospy.loginfo("Waiting for action server to start...")
        self.client.wait_for_server()
        rospy.loginfo("Action server started, sending goal.")

        rospy.sleep(1.0)

        self.pose = None

    def cable_pose_cb(self, msg):
        """Callback per aggiornare la posizione del cavo."""
        self.pose_cable = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        rospy.loginfo(f"Posizione del cavo rilevata: {self.pose_cable}")

    def publish_double(self, pub, value):
        msg = Float64()
        msg.data = value
        pub.publish(msg)

    def publish_wrench(self, pub, base_frame, force, torque):
        msg = WrenchStamped()
        msg.header.frame_id = base_frame
        msg.header.stamp = rospy.Time.now()

        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]

        msg.wrench.torque.x = torque[0]
        msg.wrench.torque.y = torque[1]
        msg.wrench.torque.z = torque[2]

        pub.publish(msg)

    def publish_pose(self, pub, base_frame, position, orientation):
        msg = PoseStamped()
        msg.header.frame_id = base_frame
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]

        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3]

        pub.publish(msg)

    def state_cb(self, state_msg):
        self.O_T_EE = np.array(state_msg.O_T_EE).reshape(4, 4).T
        O_R_EE = self.O_T_EE[:3, :3]
        position_EE = self.O_T_EE[:3, 3]
        rpy = R.from_matrix(O_R_EE).as_euler('xyz')  # Convert matrix to RPY
        self.pose = np.hstack((position_EE, rpy))

    def wrench_cb(self, msg: WrenchStamped):
        self.wrench = np.array([[msg.wrench.force.x], [msg.wrench.force.y], [msg.wrench.force.z],
                                [msg.wrench.torque.x], [msg.wrench.torque.y], [msg.wrench.torque.z]])

    def gripper_cb(self, msg: JointState):
        positions = msg.position
        dist = sum(positions)
        if dist >= 0.07:
            self.gripper_state = 0
        else:
            self.gripper_state = 1

    def load_dhb(self, path):
        data = np.load(path)
        self.positions_learn = data['positions']
        self.orientations_learn = data['orientations']
        self.duration = data['duration']

    def load_velocities(self, path):
        data = np.load(path)
        self.linear_velocities = data['velocities'][:, :3]
        self.angular_velocities = data['velocities'][:, 3:]

    def create_traj_goal(self, times, pos):
        goal = cartesian_control_msgs.msg.FollowCartesianTrajectoryGoal()
        goal.trajectory.header.frame_id = "fr3_link0"

        goal.goal_tolerance.position_error.x = 0.01
        goal.goal_tolerance.position_error.y = 0.01
        goal.goal_tolerance.position_error.z = 0.01

        goal.goal_tolerance.orientation_error.x = 0.01
        goal.goal_tolerance.orientation_error.y = 0.01
        goal.goal_tolerance.orientation_error.z = 0.01

        for i, (time, p) in enumerate(zip(times[1:], pos[1:])):
            if len(p) < 6:
                rospy.logerr(f"Pose at index {i} does not have enough elements: {len(p)}")
                continue

            traj_point = cartesian_control_msgs.msg.CartesianTrajectoryPoint()
            traj_point.time_from_start = rospy.Duration(time)
            traj_point.pose.position.x = p[0]
            traj_point.pose.position.y = p[1]
            traj_point.pose.position.z = p[2]

            traj_point.pose.orientation.x = p[4]
            traj_point.pose.orientation.y = p[5]
            traj_point.pose.orientation.z = p[6]
            traj_point.pose.orientation.w = p[3]

            traj_point.twist.linear.x = float("NaN")
            traj_point.acceleration.angular.x = float("NaN")

            goal.trajectory.points.append(traj_point)

        return goal

    def trajectory_generalization(self, position_traj, velocity_traj, start, goal):
        vector_start_end_original = position_traj[-1, :3] - position_traj[0, :3]
        vector_start_goal_desired = goal[:3] - start[:3]

        R = self.compute_rotation_matrix(vector_start_end_original, vector_start_goal_desired)

        scaling = np.linalg.norm(vector_start_goal_desired) / np.linalg.norm(vector_start_end_original)

        # Mantieni il punto di partenza fisso
        generalized_position_traj = (position_traj[:, :3] - start[:3]) * scaling
        generalized_position_traj = np.dot(generalized_position_traj, R.T) + start[:3]

        generalized_velocity_traj = np.dot(velocity_traj * scaling, R.T)

        return generalized_position_traj, generalized_velocity_traj

    def compute_rotation_matrix(self, v1, v2):
        v1_norm = v1 / np.linalg.norm(v1)
        v2_norm = v2 / np.linalg.norm(v2)

        if np.allclose(v1_norm, v2_norm):
            return np.eye(3)

        rotation_axis = np.cross(v1_norm, v2_norm)
        angle = np.arccos(np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0))
        R = self.Rodrigues(rotation_axis, angle)
        return R

    def Rodrigues(self, axis, angle):
        axis = axis / np.linalg.norm(axis)
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        I = np.eye(3)
        R = I + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)

        return R


# Funzione per plottare le traiettorie originali e generalizzate
def plot_trajectories_3d(original_positions, generalized_positions):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x_orig = original_positions[:, 0]
    y_orig = original_positions[:, 1]
    z_orig = original_positions[:, 2]

    x_gen = generalized_positions[:, 0]
    y_gen = generalized_positions[:, 1]
    z_gen = generalized_positions[:, 2]

    ax.plot(x_orig, y_orig, z_orig, label='Traiettoria originale', color='b')
    ax.plot(x_gen, y_gen, z_gen, label='Traiettoria generalizzata', color='r')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Confronto tra la Traiettoria Originale e Generalizzata')

    ax.legend()
    plt.show()


def butter_lowpass_filter(data, cutoff, fs, order=2):
    nyq = 0.5 * fs  # Frequenza di Nyquist
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data, axis=0)
    return y


def plot_trajectory_along_time(time_traj, positions):
    """Plot della traiettoria lungo gli assi X, Y e Z nel tempo."""
    plt.figure(figsize=(10, 6))

    # Plot degli assi X, Y e Z della nuova traiettoria generalizzata
    plt.plot(time_traj, positions[:, 0], label='Asse X')
    plt.plot(time_traj, positions[:, 1], label='Asse Y')
    plt.plot(time_traj, positions[:, 2], label='Asse Z')

    plt.xlabel('Tempo (s)')
    plt.ylabel('Posizione')
    plt.title('Posizione della traiettoria lungo i 3 assi nel tempo')
    plt.legend()

    # Mostra il plot
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    dhb_ros = DHB_ROS()
    dhb_ros.__init__()

    # Attendi che la posizione del cavo venga rilevata
    while dhb_ros.pose_cable is None and not rospy.is_shutdown():
        rospy.loginfo("In attesa della posizione del cavo...")
        rospy.sleep(0.1)

    rospy.loginfo(f"Posizione del cavo confermata: {dhb_ros.pose_cable}")

    slowdown_factor = 2.5

    # Carica la traiettoria e i dati di velocità
    dhb_trajectory_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/dhb/generated_traj_dhb_vel_Step_1_ee_velocities.npz'
    dhb_ros.load_dhb(dhb_trajectory_file)

    dhb_velocities_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/Step_1_ee_velocities.npz'
    dhb_ros.load_velocities(dhb_velocities_file)

    dt = dhb_ros.duration * slowdown_factor / len(dhb_ros.positions_learn)
    time_traj = np.linspace(0, dhb_ros.duration * slowdown_factor, len(dhb_ros.positions_learn))

    positions_play = dhb_ros.positions_learn
    orientations_play = dhb_ros.orientations_learn
    linear_velocities_play = dhb_ros.linear_velocities

    min_length = min(len(time_traj), len(positions_play))
    time_traj = time_traj[:min_length]
    positions_play = positions_play[:min_length]
    orientations_play = orientations_play[:min_length]
    linear_velocities_play = linear_velocities_play[:min_length]

    # x_start = positions_play[0, :3]

    # Verifica che la posizione corrente dell'EE sia disponibile
    if dhb_ros.pose is not None:
        x_start = dhb_ros.pose[:3]  # Usa la posizione corrente dell'EE
        rospy.loginfo(f"Utilizzo della posizione corrente dell'EE come punto di partenza: {x_start}")
    else:
        rospy.logerr("La posizione dell'EE non è disponibile, impossibile iniziare la traiettoria.")
        rospy.signal_shutdown("Errore: Posizione dell'EE non disponibile")



    # Definisci x_goal come la posizione del cavo rilevata
    x_goal = dhb_ros.pose_cable[:3]

    rospy.loginfo(f"Definito goal della traiettoria: {x_goal}")

    # Generalizza la traiettoria
    generalized_positions, generalized_velocities = dhb_ros.trajectory_generalization(positions_play, linear_velocities_play, x_start, x_goal)

    # Parametri per il filtro
    cutoff_frequency = 1.0  # Frequenza di taglio
    sampling_rate = dhb_ros.ros_rate  # Corrisponde a ros_rate (750 Hz)

    # Applica il filtro alle posizioni e velocità generalizzate
    filtered_positions = butter_lowpass_filter(generalized_positions, cutoff_frequency, sampling_rate)
    filtered_velocities = butter_lowpass_filter(generalized_velocities, cutoff_frequency, sampling_rate)


    plot_trajectory_along_time(time_traj, filtered_positions)  # O generalized_positions


    # Plot delle traiettorie originale e generalizzata
    plot_trajectories_3d(positions_play, filtered_positions)

    # Creazione e invio della traiettoria filtrata
    traj_points_filtered = np.hstack((filtered_positions, orientations_play))
    goal_traj_filtered = dhb_ros.create_traj_goal(time_traj, traj_points_filtered)

    dhb_ros.client.send_goal(goal_traj_filtered)
    rospy.loginfo("Goal basato sulla posizione filtrata inviato al server di azione.")

    dhb_ros.client.wait_for_result()
    result = dhb_ros.client.get_result()

    rospy.loginfo("Esecuzione della traiettoria filtrata completata con risultato: %s", result)


    dhb_ros.rate.sleep()
    rospy.sleep(2)

    if dhb_ros.pose is not None:
        goal_pose = np.hstack((generalized_positions[-1], orientations_play[-1]))
        dmp_position_error = goal_pose[:3] - generalized_positions[-1]
        control_position_error = generalized_positions[-1] - dhb_ros.pose[:3]

        print(f"DMP position error: {dmp_position_error}, norm: {np.linalg.norm(dmp_position_error)}")
        print(f"Control position error: {control_position_error}, norm: {np.linalg.norm(control_position_error)}")
        print(f"Total position error: {dmp_position_error - control_position_error}, norm: {np.linalg.norm(dmp_position_error - control_position_error)}")
    else:
        rospy.logerr("Pose is not initialized. Ensure the state_cb is being called correctly.")

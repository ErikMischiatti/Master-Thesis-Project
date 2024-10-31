#!/usr/bin/env python3

# IMPLEMENTATION OF THE trajectory_generalization FUNCTION FOR POSITION AND ORIENTATION
    # Define the function `trajectory_generalization(position_traj, velocity_traj, start, goal)` 
    # that generalizes a given trajectory for positions based on the start and goal positions.
    # The generalized trajectory will adapt to the new goal position while maintaining the relative shape of the original trajectory.
    
# IMPLEMENTATION OF ORIENTATION SCALING AND GENERALIZATION
    # Define a process for generalizing orientations. This includes converting quaternion orientations 
    # to Euler angles, applying scaling based on the difference between the start and new goal orientations,
    # and converting the scaled orientations back to quaternions.

# IMPLEMENTATION OF /cable_pose DETECTION
    # Subscribe to the `/cable_pose` topic to retrieve the position and orientation of the cable. 
    # This data will be used as the goal for both position and orientation generalization.

# FILTERING AND TRAJECTORY EXECUTION
    # Apply a low-pass filter to the generalized trajectory to ensure smooth motion.
    # Finally, create a trajectory goal and send it to the robot action server for execution.


import rospy
import rospkg
import numpy as np
import matplotlib.pyplot as plt
import actionlib
import cartesian_control_msgs.msg
import pytransform3d as p3d

from scipy.signal import butter, filtfilt
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped
from scipy.spatial.transform import Rotation as R
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from pytransform3d.rotations import matrix_from_quaternion, quaternion_from_euler, matrix_from_euler
import matplotlib.gridspec as gridspec

import os


class DHB:
    def __init__(self, dhb_file):
        # Load trajectory and velocity data
        self.load_dhb(dhb_file)

    def load_dhb(self, path):
        data = np.load(path)
        self.positions_learn = data['positions']
        self.orientations_learn = data['orientations']
        self.duration = data['duration']
        self.linear_velocities = data['linear']
        self.angular_velocities = data['angular']

    # def load_velocities(self, path):
    #     data = np.load(path)
        

    def load_predefined_traj(self):
        """Carica la traiettoria predefinita dai file."""
        try:
            # Restituisce le posizioni, orientazioni e i tempi della traiettoria
            time_traj = np.linspace(0, self.duration, len(self.positions_learn))
            return time_traj, self.positions_learn, self.orientations_learn
        except Exception as e:
            rospy.logerr(f"Errore nel caricamento della traiettoria predefinita: {e}")
            raise e

    #########################################################################################
    ###TRASLATION
    #########################################################################################
    
    def trajectory_generalization(self, position_traj, velocity_traj, start, goal):
        # print("Start genaralization", start)
        # print("goal genaralization", goal)
        vector_start_end_original = position_traj[-1, :3] - position_traj[0, :3]
        vector_start_goal_desired = goal[:3] - start[:3]

        R = self.compute_rotation_matrix(vector_start_end_original, vector_start_goal_desired)

        scaling = np.linalg.norm(vector_start_goal_desired) / np.linalg.norm(vector_start_end_original)


        # generalized_position_traj = (position_traj[:, :3] - start[:3]) * scaling
        generalized_position_traj = (position_traj[:, :3] - position_traj[0, :3]) * scaling
        generalized_position_traj = np.dot(generalized_position_traj, R.T) + start[:3]

        generalized_velocity_traj = np.dot(velocity_traj * scaling, R.T)

        # print("gerneralitzed traj last point", generalized_position_traj[-1])

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
    
    #########################################################################################
    ###ORIENTATION
    #########################################################################################
    

    def apply_scaling_per_traj(self, euler_start, euler_goal, euler_goal_prime, euler_trajectory, angular_vel_play):
        """Applica lo scaling degli angoli di Eulero su tutta la traiettoria, gestendo divisioni per zero."""
        euler_diff_original = euler_goal - euler_start
        euler_diff_new = euler_goal_prime - euler_start

        # print(f"euler_diff_original: {euler_diff_original}")
        # print(f"euler_diff_new : {euler_diff_new}")
        
        # print(f"start : {euler_start}")
        # print(f"goal : {euler_goal}")
        # print(f"goal prime : {euler_goal_prime}")
        

        # Gestione della divisione per zero: se euler_diff_original è zero, il fattore di scaling sarà 0
        scaling_factors = np.where(euler_diff_original != 0, euler_diff_new / euler_diff_original, 0)
        generalzed_vel = angular_vel_play * scaling_factors

        # scaling_factors = np.copy(np.ones_like(scaling_factors))
        # import pdb
        # pdb.set_trace()

        print(f"scale factor : {scaling_factors}")


        # import pdb
        # pdb.set_trace()

        # Applica lo scaling a ciascun punto della traiettoria
        scaled_trajectory = []
        for euler_point in euler_trajectory:
            euler_diff_point = euler_point - euler_trajectory[0]
            scaled_euler_diff = euler_diff_point * scaling_factors
            euler_final = euler_start + scaled_euler_diff
            scaled_trajectory.append(euler_final)
        
        return np.array(scaled_trajectory), scaling_factors, generalzed_vel

    #########################################################################################
    ###TRAJ. GENERALIZATION
    #########################################################################################

    def generate_traj(self, start, end, slowdown_factor, verbose=True, trajectory_name="default"):
        self.slowdown_factor = slowdown_factor
        self.dt = self.duration * slowdown_factor / len(self.positions_learn)
        time_traj = np.linspace(0, self.duration * self.slowdown_factor, len(self.positions_learn))

        # From record of traj.
        positions_play = self.positions_learn
        orientations_play = self.orientations_learn
        linear_velocities_play = self.linear_velocities
        angular_velocities_play = self.angular_velocities

        # Ensure the lengths are equal
        min_length = min(len(time_traj), len(positions_play))
        time_traj = time_traj[:min_length]
        positions_play = positions_play[:min_length]
        orientations_play = orientations_play[:min_length]
        linear_velocities_play = linear_velocities_play[:min_length]
        angular_velocities_play = angular_velocities_play[:min_length]

        orientations_play_euler = []
        for o in orientations_play:
            orientations_play_euler.append(euler_from_rotation_matrix(matrix_from_quaternion(o)))
        orientations_play_euler = np.array(orientations_play_euler)
        orientations_play_euler = np.unwrap(orientations_play_euler, axis=0)

        # import pdb
        # pdb.set_trace()

        # Start is based on the real position of EE 
        x_start = start[:3]
        x_goal = end[:3]
        # z_offset = 0.017 
        # x_goal[2] -= z_offset
        # rospy.loginfo(f"Trajectory goal defined: {x_goal}")
        # x_start = positions_play[0, :3]
        # x_goal = positions_play[-1, :3]

        ################################
        ## Generalize the trajectory ##
        ################################ 

        generalized_positions, generalized_velocities_lin = self.trajectory_generalization(positions_play, linear_velocities_play, x_start, x_goal)

        ################################
        ## Generalize orientation ##
        ################################ 
        start_orientation = start[3:]
        goal_orientation = orientations_play[-1]  
        new_goal_orientation = end[3:]

        # import pdb
        # pdb.set_trace()

        # Quaternions -> Euler angles 
        euler_start = euler_from_rotation_matrix(matrix_from_quaternion(start_orientation))
        euler_goal = euler_from_rotation_matrix(matrix_from_quaternion(goal_orientation))
        euler_new_goal = euler_from_rotation_matrix(matrix_from_quaternion(new_goal_orientation))
        euler_goal_prime = np.array([euler_goal[0], euler_goal[1], euler_new_goal[2]])
        print("euler_goal_prime", euler_goal_prime)
        # euler_goal_prime = np.array([euler_goal[0], euler_goal[1], euler_goal[2]])


        # Convert quaternions -> Euler angles
        euler_trajectory = [euler_from_rotation_matrix(matrix_from_quaternion(quat)) for quat in orientations_play]

        # Apply unwrapping to Euler angles to avoid discontinuities in rotations
        euler_trajectory_unwrapped = np.unwrap(np.array(euler_trajectory), axis=0)

        # Scaling factor 
        scaled_euler_trajectory, scaling_factors, generalized_velocities_ang = self.apply_scaling_per_traj(euler_start, euler_goal, euler_goal_prime, euler_trajectory_unwrapped, angular_velocities_play)

        # Apply unwrapping after scaling
        scaled_euler_trajectory_unwrapped = np.unwrap(scaled_euler_trajectory, axis=0)

        # import pdb
        # pdb.set_trace()

        ################################
        ## Filter the data ##
        ################################
        alpha = 0.9  # Tune alpha based on system requirements
        beta = 0.9   # Tune beta based on system requirements

        # Apply the dynamic system filter to generalized positions, velocities, and Euler trajectory
        # filtered_positions = dynamic_system_filter(generalized_positions, alpha, beta, self.dt)
        # filtered_velocities = dynamic_system_filter(generalized_velocities, alpha, beta, self.dt)
        # filtered_scaled_euler_trajectory = dynamic_system_filter(scaled_euler_trajectory_unwrapped, alpha, beta, self.dt)

        ################################
        ## Smoothing filter on filtered data ##
        ################################
        # window_length = 7  # Window length for smoothing (must be odd)
        # polyorder = 2  # Polynomial order

        # # Apply the smoothing filter to the already filtered data
        # smoothed_positions = smooth_trajectory(filtered_positions, window_length, polyorder)
        # smoothed_velocities = smooth_trajectory(filtered_velocities, window_length, polyorder)
        # smoothed_scaled_euler_trajectory = smooth_trajectory(filtered_scaled_euler_trajectory, window_length, polyorder)

        # Euler angles -> Quaternions
        filteres_scaled_orientations_quat = [quaternion_from_euler_custom(euler) for euler in scaled_euler_trajectory_unwrapped]

        # quaternion_from_euler(scaled_euler_trajectory_unwrapped[0], 0, 1, 2, extrinsic=True)

        # Adjust the quaternion length to match x_goal length
        # for pos, quat in zip(filtered_positions, filteres_scaled_orientations_quat):
        #     dhb_ros.publish_pose_with_orientation(dhb_ros.pub_des_pose, pos, quat)
        
        if verbose:
            rospy.loginfo(f"Start Orientation (Euler): {start_orientation}")
            rospy.loginfo(f"Initial Goal Orientation (Euler): {goal_orientation}")
            rospy.loginfo(f"New Goal Orientation (Euler): {new_goal_orientation}")        
            # rospy.loginfo(f"Fist position EE: {x_start}")
            # # Stampa prima della generalizzazione delle posizioni
            # rospy.loginfo(f"Original first position of the trajectory: {positions_play[0, :3]}")

            # # Stampa dopo la generalizzazione delle posizioni
            # rospy.loginfo(f"Generalized first position of the trajectory: {generalized_positions[0, :3]}")

            # Stampa prima della generalizzazione degli orientamenti
            # rospy.loginfo(f"Original first orientation (quaternion) EE: {start_orientation}")
            # rospy.loginfo(f"Original first orientation (quaternion) from record: {orientations_play[0]}")

            # Stampa dopo la generalizzazione degli orientamenti
            # rospy.loginfo(f"Generalized first orientation (Euler): {scaled_euler_trajectory[0]}")
            # rospy.loginfo(f"Generalized first orientation FILTERED (Euler): {filtered_scaled_euler_trajectory[0]}")
            # rospy.loginfo(f"Generalized last orientation (Euler): {scaled_euler_trajectory[-1]}")

            # Calcolo e stampa delle distanze
            # distance_to_first_point = np.linalg.norm(x_start - positions_play[0, :3])

            # rospy.loginfo(f"Distance from EE to first trajectory point: {distance_to_first_point:.4f}")

            # z_offset_rotation = euler_new_goal[2] - euler_goal[2]
            # rospy.loginfo(f"Cable angle: {np.degrees(euler_new_goal[2]):.4f} degrees")
            # rospy.loginfo(f"EE angle: {np.degrees(euler_goal[2]):.4f} degrees")
            # rospy.loginfo(f"Z-axis rotation offset between cable and EE last orientation: {np.degrees(z_offset_rotation):.4f} degrees")

            # import pdb
            # pdb.set_trace()
            # plot_trajectories_3d(positions_play, generalized_positions, euler_start, euler_goal, euler_new_goal, trajectory_name)
            # plot_time(time_traj, np.concatenate([positions_play, orientations_play_euler], axis=1), np.concatenate([generalized_positions, scaled_euler_trajectory_unwrapped], axis=1), np.concatenate([linear_velocities_play, angular_velocities_play], axis=1), np.concatenate([generalized_velocities_lin, generalized_velocities_ang], axis=1))

        return time_traj, generalized_positions, filteres_scaled_orientations_quat

#########################################################################################
#########################################################################################

def quaternion_from_euler_custom(euler_angles):
    """Conversione da angoli di Eulero a quaternioni."""
    return quaternion_from_euler(euler_angles, 0, 1, 2, extrinsic=True)


def euler_from_rotation_matrix(R):
        sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R[2,1], R[2,2])
            y = np.arctan2(-R[2,0], sy)
            z = np.arctan2(R[1,0], R[0,0])
        else:
            x = np.arctan2(-R[1,2], R[1,1])
            y = np.arctan2(-R[2,0], sy)
            z = 0
        return np.array([x, y, z])


def plot_trajectories_3d(original_positions, generalized_positions, euler_start, euler_goal, euler_new_goal, trajectory_name):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Estrazione delle posizioni originali e generalizzate
    x_orig = original_positions[:, 0]
    y_orig = original_positions[:, 1]
    z_orig = original_positions[:, 2]

    x_gen = generalized_positions[:, 0]
    y_gen = generalized_positions[:, 1]
    z_gen = generalized_positions[:, 2]

    # Plot delle traiettorie
    ax.plot(x_orig, y_orig, z_orig, color='b', label='Original Traj.')
    ax.plot(x_gen, y_gen, z_gen, color='r', label='Generalized Traj.')

    # Calcolo delle estensioni lungo gli assi X, Y e Z
    traj_extent_x = np.max(original_positions[:, 0]) - np.min(original_positions[:, 0])
    traj_extent_y = np.max(original_positions[:, 1]) - np.min(original_positions[:, 1])
    traj_extent_z = np.max(original_positions[:, 2]) - np.min(original_positions[:, 2])

    # Funzione per plottare gli assi e visualizzare un numero in corrispondenza del centro
    def plot_axes_from_euler(ax, origin, euler_angles, number, axis_colors=['r', 'g', 'b']):
        rotation_matrix = matrix_from_euler(euler_angles, 0, 1, 2, extrinsic=False)
        x_axis = rotation_matrix[:, 0]
        y_axis = rotation_matrix[:, 1]
        z_axis = rotation_matrix[:, 2]

        scale_x = traj_extent_x * 0.2  
        scale_y = traj_extent_y * 0.2
        scale_z = traj_extent_z * 0.2
        ax.quiver(origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2], color=axis_colors[0], length=scale_x, normalize=True)
        ax.quiver(origin[0], origin[1], origin[2], y_axis[0], y_axis[1], y_axis[2], color=axis_colors[1], length=scale_y, normalize=True)
        ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2], color=axis_colors[2], length=scale_z, normalize=True)
        ax.text(origin[0], origin[1], origin[2], f'{number}', color='black', fontsize=12, weight='bold')

    # Start
    start_position = original_positions[0, :3]
    plot_axes_from_euler(ax, start_position, euler_start, 1)

    # Goal
    goal_position = original_positions[-1, :3]
    plot_axes_from_euler(ax, goal_position, euler_goal, 2)

    # New Start
    new_start_position = generalized_positions[0, :3]
    plot_axes_from_euler(ax, new_start_position, euler_start, 3, axis_colors=['m', 'y', 'c'])

    # New Goal
    new_goal_position = generalized_positions[-1, :3]
    plot_axes_from_euler(ax, new_goal_position, euler_new_goal, 4, axis_colors=['m', 'y', 'c'])

    # Aggiungi i punti neri con i nomi nella legenda
    ax.scatter(start_position[0], start_position[1], start_position[2], color='red', s=50, label='1 = Start')
    ax.scatter(goal_position[0], goal_position[1], goal_position[2], color='green', s=50, label='2 = Goal')
    ax.scatter(new_start_position[0], new_start_position[1], new_start_position[2], color='blue', s=50, label='3 = Start Prime')
    ax.scatter(new_goal_position[0], new_goal_position[1], new_goal_position[2], color='purple', s=50, label='4 = Goal Prime')


    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Calcolo della scala uniforme per tutti gli assi
    max_range = np.array([traj_extent_x, traj_extent_y, traj_extent_z]).max() / 2.0

    mid_x = (np.max(original_positions[:, 0]) + np.min(original_positions[:, 0])) * 0.5
    mid_y = (np.max(original_positions[:, 1]) + np.min(original_positions[:, 1])) * 0.5
    mid_z = (np.max(original_positions[:, 2]) + np.min(original_positions[:, 2])) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Legenda per le traiettorie e i punti centrali degli assi con i numeri 1, 2, 3, 4
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1))
    fig.subplots_adjust(right=0.75)

    # Funzione per aggiornare la vista del grafico (ruota il grafico)
    def update(frame):
        ax.view_init(elev=30, azim=frame)

    # Crea l'animazione con rotazione automatica
    anim = FuncAnimation(fig, update, frames=np.arange(0, 360, 2), interval=50)

    fig.tight_layout()

    #GIF
    output_directory = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/gif/' 
    gif_filename = f'{output_directory}_{trajectory_name}.gif'
    anim.save(gif_filename, writer='imagemagick', fps=20)

    # plt.show()




def integrate_trajectory(w, dt, initial_orientation):
    from pytransform3d.rotations import quaternion_integrate
    num_points = w.shape[0]

    # Integrate angular velocities to obtain quaternions
    quaternions = quaternion_integrate(w, q0=initial_orientation, dt=dt)
    orientations = np.array([quaternion for quaternion in quaternions])

    orientations_play_euler = []
    for o in orientations:
        orientations_play_euler.append(euler_from_rotation_matrix(matrix_from_quaternion(o)))
    orientations_play_euler = np.array(orientations_play_euler)
    orientations_play_euler = np.unwrap(orientations_play_euler, axis=0)


    return orientations_play_euler

def dynamic_system_filter(raw_signal, alpha, beta, dt):
    """
    Filtro dinamico di secondo ordine basato su un sistema dinamico.
    
    raw_signal: il segnale grezzo da filtrare (ad es. posizioni o velocità)
    alpha: costante di scala per il filtro
    beta: costante di scala per il filtro
    dt: passo temporale tra le iterazioni
    
    Restituisce: il segnale filtrato
    """
    # Inizializza il segnale filtrato con il primo valore del segnale grezzo
    filtered_signal = np.zeros_like(raw_signal)
    filtered_signal[0] = raw_signal[0]  # Il filtro inizia dal primo valore del segnale grezzo
    
    # Inizializza la velocità (derivata prima) e l'accelerazione (derivata seconda) a zero
    signal_dot = np.zeros_like(raw_signal)
    signal_ddot = np.zeros_like(raw_signal)
    
    # Itera attraverso ciascun campione del segnale
    for i in range(1, len(raw_signal)):
        # Calcola la differenza tra il segnale grezzo e quello filtrato
        signal_diff = raw_signal[i] - filtered_signal[i - 1]
        
        # Applica il filtro dinamico usando l'equazione fornita
        signal_ddot[i] = alpha * (beta * signal_diff - signal_dot[i - 1])
        
        # Calcola la derivata prima e aggiorna il segnale filtrato
        signal_dot[i] = signal_dot[i - 1] + signal_ddot[i] * dt
        filtered_signal[i] = filtered_signal[i - 1] + signal_dot[i] * dt
    
    return filtered_signal

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


def plot_orientations(time_traj, original_orientations_degrees, scaled_orientations_degrees, original_orientations_quat, scaled_orientations_quat):
    """Plotta quaternioni ed Euler originali e riscalati lungo il tempo, e visualizza fattori di scaling e coordinate di start/goal."""

    # Crea una griglia con 4 righe (una per ciascun plot)
    fig = plt.figure(figsize=(12, 16))
    gs = gridspec.GridSpec(4, 1, height_ratios=[1, 1, 1, 1])

    # Plot dei quaternioni originali
    ax1 = plt.subplot(gs[0])

    ax1.plot(time_traj, [quat[0] for quat in original_orientations_quat], label='q_x originale', color='r', linestyle='--')
    ax1.plot(time_traj, [quat[1] for quat in original_orientations_quat], label='q_y originale', color='g', linestyle='--')
    ax1.plot(time_traj, [quat[2] for quat in original_orientations_quat], label='q_z originale', color='b', linestyle='--')
    ax1.plot(time_traj, [quat[3] for quat in original_orientations_quat], label='q_w originale', color='k', linestyle='--')
    ax1.set_xlabel('Tempo (s)')
    ax1.set_ylabel('Quaternioni')
    ax1.set_title('Quaternioni Originali')
    ax1.legend()
    ax1.grid(True)

    # Plot degli angoli di Eulero originali
    ax2 = plt.subplot(gs[1])
    ax2.plot(time_traj, original_orientations_degrees[:, 0], label='Euler X originale', color='r', linestyle='--')
    ax2.plot(time_traj, original_orientations_degrees[:, 1], label='Euler Y originale', color='g', linestyle='--')
    ax2.plot(time_traj, original_orientations_degrees[:, 2], label='Euler Z originale', color='b', linestyle='--')
    ax2.set_xlabel('Tempo (s)')
    ax2.set_ylabel('Angoli di Eulero (gradi)')
    ax2.set_title('Angoli di Eulero Originali')
    ax2.legend()
    ax2.grid(True)

    # Plot degli angoli di Eulero scalati
    ax3 = plt.subplot(gs[2])
    ax3.plot(time_traj, scaled_orientations_degrees[:, 0], label='Euler X scalato', color='r')
    ax3.plot(time_traj, scaled_orientations_degrees[:, 1], label='Euler Y scalato', color='g')
    ax3.plot(time_traj, scaled_orientations_degrees[:, 2], label='Euler Z scalato', color='b')
    ax3.set_xlabel('Tempo (s)')
    ax3.set_ylabel('Angoli di Eulero Scalati (gradi)')
    ax3.set_title('Angoli di Eulero Scalati lungo la traiettoria')
    ax3.legend()
    ax3.grid(True)

    # Plot dei quaternioni scalati
    ax4 = plt.subplot(gs[3])
    ax4.plot(time_traj, [quat[0] for quat in scaled_orientations_quat], label='q_x scalato', color='r')
    ax4.plot(time_traj, [quat[1] for quat in scaled_orientations_quat], label='q_y scalato', color='g')
    ax4.plot(time_traj, [quat[2] for quat in scaled_orientations_quat], label='q_z scalato', color='b')
    ax4.plot(time_traj, [quat[3] for quat in scaled_orientations_quat], label='q_w scalato', color='k')
    ax4.set_xlabel('Tempo (s)')
    ax4.set_ylabel('Quaternioni Scalati')
    ax4.set_title('Quaternioni Scalati lungo la traiettoria')
    ax4.legend()
    ax4.grid(True)

    # # Sezione legenda personalizzata
    # ax4 = plt.subplot(gs[3])
    # ax4.axis('off')  # Nasconde gli assi
    # legenda_text = (
    #     f"Fattori di scaling (X, Y, Z): {scaling_factors.round(3)}\n"
    #     f"Start Euler (gradi): {np.degrees(euler_start).round(3)}\n"
    #     f"Goal Original Euler (gradi): {np.degrees(euler_goal).round(3)}\n"
    #     f"New Goal Euler (gradi): {np.degrees(euler_new_goal).round(3)}"
    # )
    # ax4.text(0.05, 0.5, legenda_text, ha='left', va='center', fontsize=10, color='black', family='monospace')

    # Regola i layout per evitare sovrapposizioni
    plt.tight_layout()
    plt.show()

def is_valid_quaternion(orientation):
    # Rileva il quaternione predefinito [0, 0, 0, 1] che rappresenta "nessuna rotazione"
    return not (orientation[3] == 0 and orientation[1] == 0 and orientation[2] == 0 and orientation[0] == 1) 


##################################################################################
##################################################################################motion succesful p:

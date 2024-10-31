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
from pytransform3d.rotations import matrix_from_quaternion, quaternion_from_euler, matrix_from_euler
import matplotlib.gridspec as gridspec




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

    # MOD.  
    def cable_pose_cb(self, msg):
        """Callback per aggiornare la posizione e l'orientamento del cavo."""
        self.pose_cable = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.orientation_cable = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        # rospy.loginfo(f"Posizione del cavo rilevata: {self.pose_cable}")
        # rospy.loginfo(f"Orientamento del cavo rilevato: {self.orientation_cable}")


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
        self.O_T_EE = np.array(state_msg.O_T_EE).reshape(4, 4).T  # Matrice di trasformazione 4x4
        O_R_EE = self.O_T_EE[:3, :3]  # Estrazione della matrice di rotazione 3x3
        position_EE = self.O_T_EE[:3, 3]  # Estrazione della posizione (prime 3 righe della quarta colonna)

        # Conversione della matrice di rotazione in angoli di Eulero
        euler_angles = euler_from_rotation_matrix(O_R_EE)  # Usa la tua funzione personalizzata

        # Conversione degli angoli di Eulero in quaternioni
        orientation_quat = quaternion_from_euler_custom(euler_angles)  # Usa la tua funzione per convertire da Eulero a quaternioni

        # Combina la posizione e gli angoli di Eulero in un unico array (pose con orientamento in Eulero)
        self.pose = np.hstack((position_EE, euler_angles))  # Posizione + Orientamento in angoli di Eulero

        # Salva il quaternione dell'end-effector
        self.orientation_ee_quat = orientation_quat  # Orientamento in quaternione



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

    #########################################################################################
    ###TRASLATION
    #########################################################################################
    
    def trajectory_generalization(self, position_traj, velocity_traj, start, goal):
        vector_start_end_original = position_traj[-1, :3] - position_traj[0, :3]
        vector_start_goal_desired = goal[:3] - start[:3]

        R = self.compute_rotation_matrix(vector_start_end_original, vector_start_goal_desired)

        scaling = np.linalg.norm(vector_start_goal_desired) / np.linalg.norm(vector_start_end_original)

        # Mantieni il punto di partenza fisso
        generalized_position_traj = (position_traj[:, :3] - position_traj[0, :3]) * scaling
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
    
    #########################################################################################
    ###ORIENTATION
    #########################################################################################
    

    def apply_scaling_per_traj(self, euler_start, euler_goal, euler_goal_prime, euler_trajectory):
        """Applica lo scaling degli angoli di Eulero su tutta la traiettoria, gestendo divisioni per zero."""
        euler_diff_original = euler_goal - euler_start
        euler_diff_new = euler_goal_prime - euler_start

        print(f"euler_diff_original: {euler_diff_original}")
        print(f"euler_diff_new : {euler_diff_new}")
        
        print(f"start : {euler_start}")
        print(f"goal : {euler_goal}")
        print(f"goal prime : {euler_goal_prime}")
        

        # Gestione della divisione per zero: se euler_diff_original è zero, il fattore di scaling sarà 0
        scaling_factors = np.where(euler_diff_original != 0, euler_diff_new / euler_diff_original, 0)

        print(f"scale factor : {scaling_factors}")

        # Applica lo scaling a ciascun punto della traiettoria
        scaled_trajectory = []
        for euler_point in euler_trajectory:
            euler_diff_point = euler_point - euler_start
            scaled_euler_diff = euler_diff_point * scaling_factors
            euler_final = euler_start + scaled_euler_diff
            scaled_trajectory.append(euler_final)
        
        return np.array(scaled_trajectory), scaling_factors

    

    def publish_pose_with_orientation(self, pub, position, orientation_quat):
        """Pubblica la posa con l'orientamento aggiornato."""
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "robot_base"
        
        # Imposta la posizione
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        
        # Imposta l'orientamento in quaternioni
        msg.pose.orientation.x = orientation_quat[0]
        msg.pose.orientation.y = orientation_quat[1]
        msg.pose.orientation.z = orientation_quat[2]
        msg.pose.orientation.w = orientation_quat[3]
        
        pub.publish(msg)


#########################################################################################
#########################################################################################

def quaternion_from_euler_custom(euler_angles):
    """Conversione da angoli di Eulero a quaternioni."""
    return quaternion_from_euler(euler_angles, 0, 1, 2, extrinsic=False)


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


def plot_trajectories_3d(original_positions, generalized_positions, euler_start, euler_goal, euler_new_goal):
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
    ax.plot(x_orig, y_orig, z_orig, label='Traiettoria originale', color='b')
    ax.plot(x_gen, y_gen, z_gen, label='Traiettoria generalizzata', color='r')

    # Funzione per ottenere gli assi da angoli di Eulero usando pytransform3d
    def plot_axes_from_euler(ax, origin, euler_angles, label):
        # Converte gli angoli di Eulero in una matrice di rotazione usando pytransform3d
        rotation_matrix = matrix_from_euler(euler_angles, 0, 1, 2, extrinsic=False) 
        
        # Assi locali (unitari)
        x_axis = rotation_matrix[:, 0]  # Primo vettore colonna
        y_axis = rotation_matrix[:, 1]  # Secondo vettore colonna
        z_axis = rotation_matrix[:, 2]  # Terzo vettore colonna
        
        # Moltiplica per una costante per scalare la visualizzazione degli assi
        scale = 0.05
        ax.quiver(origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2], color='r', length=scale, normalize=True, label=f'{label} X')
        ax.quiver(origin[0], origin[1], origin[2], y_axis[0], y_axis[1], y_axis[2], color='g', length=scale, normalize=True, label=f'{label} Y')
        ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2], color='b', length=scale, normalize=True, label=f'{label} Z')

    # Plot degli assi relativi alla posa iniziale
    start_position = original_positions[0, :3]
    plot_axes_from_euler(ax, start_position, euler_start, 'Start')

    # Plot degli assi relativi al goal originale
    goal_position = original_positions[-1, :3]
    plot_axes_from_euler(ax, goal_position, euler_goal, 'Goal')

    # Plot degli assi relativi al nuovo goal (generalizzato)
    new_goal_position = generalized_positions[-1, :3]
    plot_axes_from_euler(ax, new_goal_position, euler_new_goal, 'Goal Prime')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Confronto tra la Traiettoria Originale e Generalizzata con Orientamenti')

    ax.legend()
    plt.show()




# def butter_lowpass_filter(data, cutoff, fs, order=2):
#     """
#     Applica un filtro passa-basso sui dati.
#     data: i dati che vuoi filtrare (posizioni o velocità)
#     cutoff: la frequenza di taglio del filtro (in Hz)
#     fs: frequenza di campionamento (in Hz)
#     order: ordine del filtro (di solito 2 o 3 è sufficiente)
#     """
#     nyq = 0.5 * fs  # Frequenza di Nyquist
#     normal_cutoff = cutoff / nyq  # Frequenza di taglio normalizzata
#     b, a = butter(order, normal_cutoff, btype='low', analog=False)
#     y = filtfilt(b, a, data, axis=0)
#     return y

# from scipy.signal import savgol_filter

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





# def smooth_trajectory(trajectory, window_length, polyorder):
#     """
#     Applica un filtro di smoothing alla traiettoria usando Savitzky-Golay.
#     trajectory: array di posizioni (Nx3) o velocità.
#     window_length: lunghezza della finestra per il filtro (deve essere dispari).
#     polyorder: ordine del polinomio da usare per lo smoothing.
#     """
#     smoothed_traj = savgol_filter(trajectory, window_length, polyorder, axis=0)
#     return smoothed_traj



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
##################################################################################

if __name__ == '__main__':
    dhb_ros = DHB_ROS()

    # Wait until the cable position is detected
    while dhb_ros.pose_cable is None or not is_valid_quaternion(dhb_ros.orientation_cable) and not rospy.is_shutdown():
        rospy.loginfo("Waiting for valid cable position and orientation...")
        rospy.sleep(5)

    rospy.loginfo(f"CABLE POSITION CONFIRMED: {dhb_ros.pose_cable}")
    rospy.loginfo(f"CABLE ORIENTATION CONFIRMED: {dhb_ros.orientation_cable}")

    slowdown_factor = 2

    # Load trajectory and velocity data
    dhb_trajectory_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/dhb/generated_traj_dhb_vel_approach_cable_2_ee_velocities.npz'
    dhb_ros.load_dhb(dhb_trajectory_file)

    dhb_velocities_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/approach_cable_2_ee_velocities.npz'
    dhb_ros.load_velocities(dhb_velocities_file)

    dt = dhb_ros.duration * slowdown_factor / len(dhb_ros.positions_learn)
    time_traj = np.linspace(0, dhb_ros.duration * slowdown_factor, len(dhb_ros.positions_learn))

    positions_play = dhb_ros.positions_learn
    orientations_play = dhb_ros.orientations_learn
    linear_velocities_play = dhb_ros.linear_velocities

    # Ensure the lengths are equal
    min_length = min(len(time_traj), len(positions_play))
    time_traj = time_traj[:min_length]
    positions_play = positions_play[:min_length]
    orientations_play = orientations_play[:min_length]
    linear_velocities_play = linear_velocities_play[:min_length]

    # Verify if the current EE position is available
    if dhb_ros.pose is not None:
        x_start = dhb_ros.pose[:3]  # Use the current EE position
        rospy.loginfo(f"Using the current EE position as the start point: {x_start}")
    else:
        rospy.logerr("The EE position is not available, unable to start trajectory.")
        rospy.signal_shutdown("Error: EE position not available")

    # Define x_goal as the detected cable position
    x_goal = dhb_ros.pose_cable[:3]
    z_offset = 0.02 
    x_goal[2] -= z_offset
    rospy.loginfo(f"Trajectory goal defined: {x_goal}")

    ################################
    ## Generalize the trajectory ##
    ################################ 

    generalized_positions, generalized_velocities = dhb_ros.trajectory_generalization(positions_play, linear_velocities_play, x_start, x_goal)

    ################################
    ## Generalize orientation ##
    ################################ 
    start_orientation = dhb_ros.orientation_ee_quat  
    start_orientation = orientations_play[0]
    goal_orientation = orientations_play[-1]  
    new_goal_orientation = dhb_ros.orientation_cable  

    # Quaternions -> Euler angles 
    rotation_offset = + np.pi/2
    euler_start = euler_from_rotation_matrix(matrix_from_quaternion(start_orientation))
    euler_goal = euler_from_rotation_matrix(matrix_from_quaternion(goal_orientation))
    euler_new_goal = euler_from_rotation_matrix(matrix_from_quaternion(new_goal_orientation))
    euler_goal_prime = np.array([euler_goal[0], euler_goal[1], euler_new_goal[2] + rotation_offset])

    # Convert quaternions -> Euler angles
    euler_trajectory = [euler_from_rotation_matrix(matrix_from_quaternion(quat)) for quat in orientations_play]

    # Apply unwrapping to Euler angles to avoid discontinuities in rotations
    euler_trajectory_unwrapped = np.unwrap(np.array(euler_trajectory), axis=0)

    # Scaling factor 
    scaled_euler_trajectory, scaling_factors = dhb_ros.apply_scaling_per_traj(euler_start, euler_goal, euler_goal_prime, euler_trajectory_unwrapped)

    # Apply unwrapping after scaling
    scaled_euler_trajectory_unwrapped = np.unwrap(scaled_euler_trajectory, axis=0)

    ################################
    ## Filter the data ##
    ################################
    # cutoff_frequency = 1  # Cutoff frequency
    # sampling_rate = dhb_ros.ros_rate  # Corresponds to ros_rate (750 Hz)

    # # Apply the filter to generalized positions and velocities
    # filtered_positions = butter_lowpass_filter(generalized_positions, cutoff_frequency, sampling_rate)
    # filtered_velocities = butter_lowpass_filter(generalized_velocities, cutoff_frequency, sampling_rate)
    # filtered_scaled_euler_trajectory = butter_lowpass_filter(scaled_euler_trajectory_unwrapped, cutoff_frequency, sampling_rate)

    alpha = 0.8  # Tune alpha based on system requirements
    beta = 0.3   # Tune beta based on system requirements

    # Apply the dynamic system filter to generalized positions, velocities, and Euler trajectory
    filtered_positions = dynamic_system_filter(generalized_positions, alpha, beta, dt)
    filtered_velocities = dynamic_system_filter(generalized_velocities, alpha, beta, dt)
    filtered_scaled_euler_trajectory = dynamic_system_filter(scaled_euler_trajectory_unwrapped, alpha, beta, dt)



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
    filteres_scaled_orientations_quat = [quaternion_from_euler_custom(euler) for euler in filtered_scaled_euler_trajectory]

    # Adjust the quaternion length to match x_goal length
    for pos, quat in zip(filtered_positions, filteres_scaled_orientations_quat):
        dhb_ros.publish_pose_with_orientation(dhb_ros.pub_des_pose, pos, quat)

    ################################
    ## PLOTS ##
    ################################
    original_orientations_degrees = np.degrees(euler_trajectory_unwrapped)
    scaled_orientations_degrees = np.degrees(filtered_scaled_euler_trajectory)

    plot_trajectory_along_time(time_traj, filtered_positions)
    plot_trajectory_along_time(time_traj, generalized_positions)
    plot_trajectories_3d(positions_play, filtered_positions, euler_start, euler_goal, euler_goal_prime)
    plot_orientations(time_traj, original_orientations_degrees, scaled_orientations_degrees, orientations_play, filteres_scaled_orientations_quat)

    ################################
    ## Create and send the filtered trajectory ##
    ################################
    traj_points_filtered = [np.hstack((pos, quat)) for pos, quat in zip(filtered_positions, filteres_scaled_orientations_quat)]
    goal_traj_filtered = dhb_ros.create_traj_goal(time_traj, traj_points_filtered)

    dhb_ros.client.send_goal(goal_traj_filtered)
    rospy.loginfo("Filtered position goal sent to the action server.")

    dhb_ros.client.wait_for_result()
    result = dhb_ros.client.get_result()

    rospy.loginfo("Filtered trajectory execution completed with result: %s", result)

    dhb_ros.rate.sleep()
    rospy.sleep(2)

    if dhb_ros.pose is not None:
        goal_pose = np.hstack((generalized_positions[-1], orientations_play[-1]))
        dmp_position_error = goal_pose[:3] - generalized_positions[-1]
        control_position_error = generalized_positions[-1] - dhb_ros.pose[:3]

        print(f"DHB position error: {dmp_position_error}, norm: {np.linalg.norm(dmp_position_error)}")
        print(f"Control position error: {control_position_error}, norm: {np.linalg.norm(control_position_error)}")
        print(f"Total position error: {dmp_position_error - control_position_error}, norm: {np.linalg.norm(dmp_position_error - control_position_error)}")
    else:
        rospy.logerr("Pose is not initialized. Ensure the state_cb is being called correctly.")



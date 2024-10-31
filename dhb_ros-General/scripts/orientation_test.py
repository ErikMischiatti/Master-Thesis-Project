#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D
from pytransform3d.rotations import matrix_from_quaternion

class DHB_ROS:
    def __init__(self):
        self.positions_learn = None
        self.orientations_learn = None
        self.duration = None

    def load_dhb(self, path):
        """Carica la traiettoria dal file .npz e verifica il formato dei quaternioni"""
        data = np.load(path)
        self.positions_learn = data['positions']  # Posizioni della traiettoria

        self.orientations_learn = data['orientations']
        
    
        # orientations = data['orientations']
    
        # Controlla quale formato è corretto, [x, y, z, w] o [w, x, y, z]
        # self.orientations_learn = check_quaternion_format(orientations)
        # self.orientations_learn = np.array([[o[3], o[0], o[1], o[2]] for o in orientations])  # Converti in [w, x, y, z]

        self.duration = data['duration']  # Durata della traiettoria

    def apply_scaling_per_traj(self, euler_start, euler_goal, euler_goal_prime, euler_trajectory):
        """Applica lo scaling degli angoli di Eulero su tutta la traiettoria, gestendo divisioni per zero."""
        euler_diff_original = euler_goal - euler_start
        euler_diff_new = euler_goal_prime - euler_start

        # Gestione della divisione per zero: se euler_diff_original è zero, il fattore di scaling sarà 0
        scaling_factors = np.where(euler_diff_original != 0, euler_diff_new / euler_diff_original, 0)

        # Applica lo scaling a ciascun punto della traiettoria
        scaled_trajectory = []
        for euler_point in euler_trajectory:
            euler_diff_point = euler_point - euler_start
            scaled_euler_diff = euler_diff_point * scaling_factors
            euler_final = euler_start + scaled_euler_diff
            scaled_trajectory.append(euler_final)
        
        return np.array(scaled_trajectory), scaling_factors



def euler_from_rotation_matrix(R):
    """Converte una matrice di rotazione in angoli di Eulero."""
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])


import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def plot_quaternions_along_time(time_traj, original_orientations_quat, quat_goal_prime):
    """Plotta i quaternioni e confronta gli angoli di Eulero originali con quelli scalati."""
    
    # Estrai i componenti dei quaternioni
    original_orientations_quat = np.array(original_orientations_quat)

    # Placeholder per gli angoli di Eulero
    euler_time_series2 = []

    # Itera sui quaternioni e convertili in matrici di rotazione, poi in angoli di Eulero
    for quat in original_orientations_quat:
        rot_matrix = matrix_from_quaternion(quat)
        euler_angles = euler_from_rotation_matrix(rot_matrix)
        euler_time_series2.append(euler_angles)

    # Converti in un array numpy per una gestione più semplice
    euler_time_series1 = np.array(euler_time_series2)

    # Applica "unwrap" per evitare discontinuità negli angoli di Eulero
    euler_time_series = np.unwrap(np.copy(euler_time_series1), axis=0)

    # Converti gli angoli di Eulero da radianti a gradi
    euler_time_series_degrees = np.degrees(euler_time_series)

    # Estrai gli angoli iniziali e finali dalla traiettoria originale
    euler_start = euler_time_series_degrees[0]
    euler_goal = euler_time_series_degrees[-1]

    # Converte il nuovo goal in angoli di Eulero
    rot_matrix_goal_prime = matrix_from_quaternion(quat_goal_prime)
    euler_goal_prime2 = euler_from_rotation_matrix(rot_matrix_goal_prime)
    
    # Converte il nuovo goal prime in gradi
    euler_goal_prime2_degrees = np.degrees(euler_goal_prime2)

    # Seleziona roll e pitch dal goal originale e modifica solo il yaw
    euler_goal_prime = np.array([euler_goal[0], euler_goal[1], euler_goal_prime2_degrees[2]])

    # Applica lo scaling sugli angoli di Eulero (radianti per coerenza interna se necessario)
    dhb_ros = DHB_ROS()  # Crea un'istanza della classe
    scaled_trajectory, scaling_factors = dhb_ros.apply_scaling_per_traj(np.radians(euler_start), np.radians(euler_goal), np.radians(euler_goal_prime), euler_time_series2
    )
    
    # Converti la traiettoria scalata da radianti a gradi
    scaled_trajectory_degrees = np.degrees(np.unwrap(scaled_trajectory, axis=0))

    print(f"Fattori di scaling: {scaling_factors}")
    print(f"Euler start: {euler_start}")
    print(f"Euler goal: {euler_goal}")
    print(f"Euler goal prime: {euler_goal_prime}")

    # Plot dei componenti dei quaternioni originali
    plt.subplot(3, 1, 1)
    plt.plot(time_traj, original_orientations_quat[:, 0], label='q_x original', color='r', linestyle='--')
    plt.plot(time_traj, original_orientations_quat[:, 1], label='q_y original', color='g', linestyle='--')
    plt.plot(time_traj, original_orientations_quat[:, 2], label='q_z original', color='b', linestyle='--')
    plt.plot(time_traj, original_orientations_quat[:, 3], label='q_w original', color='k', linestyle='--')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Quaternioni')
    plt.title('Componenti del quaternione originale nel tempo')
    plt.legend()
    plt.grid(True)

    # Plot degli angoli di Eulero originali
    plt.subplot(3, 1, 2)
    plt.plot(time_traj, euler_time_series_degrees[:, 0], label='Euler X originale', color='r', linestyle='--')
    plt.plot(time_traj, euler_time_series_degrees[:, 1], label='Euler Y originale', color='g', linestyle='--')
    plt.plot(time_traj, euler_time_series_degrees[:, 2], label='Euler Z originale', color='b', linestyle='--')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Angoli di Eulero (gradi)')
    plt.title('Angoli di Eulero originali')
    plt.legend()
    plt.grid(True)

    # Plot degli angoli di Eulero scalati lungo tutta la traiettoria
    plt.subplot(3, 1, 3)
    plt.plot(time_traj, scaled_trajectory_degrees[:, 0], label='Euler X scalato', color='r')
    plt.plot(time_traj, scaled_trajectory_degrees[:, 1], label='Euler Y scalato', color='g')
    plt.plot(time_traj, scaled_trajectory_degrees[:, 2], label='Euler Z scalato', color='b')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Angoli di Eulero Scalati (gradi)')
    plt.title('Angoli di Eulero Scalati lungo la traiettoria')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()



def check_quaternion_format(orientations):
    """Controlla se i quaternioni sono nel formato [x, y, z, w] o [w, x, y, z]"""
    
    # Testa con l'assunzione che i dati siano nel formato [x, y, z, w]
    orientations_xyzw = orientations  # Lascia i dati così come sono
    
    # Testa con l'assunzione che i dati siano nel formato [w, x, y, z]
    orientations_wxyz = np.array([[o[3], o[0], o[1], o[2]] for o in orientations])  # Riordina i dati

    # Converti in angoli di Eulero (assumendo che le rotazioni siano espresse in quaternioni)
    euler_angles_xyzw = np.array([R.from_quat(o).as_euler('xyz', degrees=True) for o in orientations_xyzw])
    euler_angles_wxyz = np.array([R.from_quat(o).as_euler('xyz', degrees=True) for o in orientations_wxyz])

    # Stampa i risultati per il confronto
    print("Angoli di Eulero (formato xyzw):", euler_angles_xyzw[:5])  # Solo primi 5 per esempio
    print("Angoli di Eulero (formato wxyz):", euler_angles_wxyz[:5])  # Solo primi 5 per esempio

    # Controlla quale insieme di angoli ha differenze più piccole e continue
    # La differenza tra gli angoli dovrebbe essere piccola per traiettorie corrette
    diff_xyzw = np.abs(np.diff(euler_angles_xyzw, axis=0)).max()
    diff_wxyz = np.abs(np.diff(euler_angles_wxyz, axis=0)).max()

    print(f"Max differenza (xyzw): {diff_xyzw}, Max differenza (wxyz): {diff_wxyz}")

    # Ritorna il formato con le differenze più piccole
    return orientations_xyzw if diff_xyzw < diff_wxyz else orientations_wxyz



def test_orientation_generalization():
    # Inizializza l'oggetto DHB_ROS
    dhb_ros = DHB_ROS()
    dhb_ros.__init__()

    slowdown_factor = 2

    # Carica una traiettoria di esempio
    dhb_trajectory_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/dhb/generated_traj_dhb_vel_Step_1_ee_velocities.npz'
    dhb_ros.load_dhb(dhb_trajectory_file)

    # Definisci un nuovo goal di orientamento manualmente
    new_goal_orientation_quat = np.array([0.707, 0.0, 0.0, 0.707]) 

    # Definisci il tempo per la traiettoria
    time_traj = np.linspace(0, dhb_ros.duration * slowdown_factor, len(dhb_ros.positions_learn))

    # Plotta i quaternioni lungo il tempo e applica lo scaling sugli angoli di Eulero
    plot_quaternions_along_time(time_traj, dhb_ros.orientations_learn, new_goal_orientation_quat)


if __name__ == '__main__':
    test_orientation_generalization()

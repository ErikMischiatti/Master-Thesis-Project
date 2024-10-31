# Author: Matteo Saveriano - 01.03.18

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from computeDHB import computeDHB
from dhb_ros.reconstructTrajectory import reconstructTrajectory



# Parameters (Construct a 6DOF trajectory)
dt = 0.1  # sample time
tf = 4    # total time
N = 1000  # number of samples 

# Time vector
T = np.linspace(0, tf, N)

# 6DOF trajectory
x = 0.1 * np.exp(T)
y = 5 + 1.5 * np.sin(T)
z = np.cos(T)


roll = 0.5 * np.sin(T)
pitch = np.cos(T)
yaw = 0.1 * T


positions = np.vstack((x, y, z)).T
orientations = np.vstack((roll, pitch, yaw)).T

# # Position and orientation shape
# print("Position shape:", positions.shape)
# print("Orientation shape:", orientations.shape)

# Orientations print for an overview
print("Orientations:")
for row in orientations[:5]:
    print(" ".join(f"{value:.15f}" for value in row))
print("##############################################")


# orientations = normalize_angles(orientations)

# print(f"x: {[f'{num:.15f}' for num in x[:5]]}")
# print(f"y: {[f'{num:.15f}' for num in y[:5]]}")
# print(f"z: {[f'{num:.15f}' for num in z[:5]]}")
# print(f"roll: {[f'{num:.15f}' for num in roll[:5]]}")
# print(f"pitch: {[f'{num:.15f}' for num in pitch[:5]]}")
# print(f"yaw: {[f'{num:.15f}' for num in yaw[:5]]}")
# print("##############################################")

# Method to compute DHB invariants ('pos' or 'vel')
method = 'pos' # 'pos' or 'vel'


# Compute velocity (twist)
velocities = np.diff(positions, axis=0) / dt
orient_rates = np.diff(orientations, axis=0) / dt

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
    
# Stampa dei twist
# print("Twists:", twists[:5])
# print("Twists:")
# for row in twists[:5]:
#     print(" ".join(f"{value:.15f}" for value in row))
# print("##############################################")


if method == 'pos':

    delta_positions = np.diff(positions, axis=0)
    delta_orientations = np.diff(orientations, axis=0)

    # Adjustment for orientation rates
    delta_orientations = np.zeros((N-1, 3))
    for i in range(N-1):
        Tr = np.array([
            [1, 0, -np.sin(pitch[i])],
            [0, np.cos(roll[i]), np.cos(pitch[i]) * np.sin(roll[i])],
            [0, -np.sin(roll[i]), np.cos(pitch[i]) * np.cos(roll[i])]
        ])
        delta_orientations[i] = (Tr @ orient_rates[i, :].T).T
        

    # pose0 = [x[0],y[0],z[0]]
    pose0 = positions[0]
    # pose0 = [0,0,0]
    invariants, Hv0, Hw0 = computeDHB(delta_positions, delta_orientations, method, pose0)
else:
    invariants, Hv0, Hw0 = computeDHB(twists[:, 3:], twists[:, :3], method)


# Stampa degli invarianti e dei frame iniziali
print("Invariants:")
for row in invariants[:5]:
    print(" ".join(f"{value:.15f}" for value in row))
print("##############################################")
print("Initial Linear Frame (Hv0):")
for row in Hv0:
    print(" ".join(f"{value:.15f}" for value in row))
print("##############################################")
print("Initial Angular Frame (Hw0):")
for row in Hw0:
    print(" ".join(f"{value:.15f}" for value in row))
print("##############################################")


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

# Reconstruct original trajectory
vr, wr = reconstructTrajectory(invariants, Hv0, Hw0, method)

# Stampa della ricostruzione
# print("Reconstructed Linear Velocity or Position (v):")
# for row in vr[:5]:
#     print(" ".join(f"{value:.15f}" for value in row))
# print("##############################################")
# print("Reconstructed Angular Velocity or Rotation Vector (w):")
# for row in wr[:5]:
#     print(" ".join(f"{value:.15f}" for value in row))
# print("##############################################")


# Indice di interesse
# index = 2 
# print("Original at index:", twists[index, 3:6])
# print("Reconstructed at index:", vr[index] if index < vr.shape[0] else "Index out of bounds for vr")
# print("Original at index-1:", twists[index-1, 3:6] if index > 0 else "Index 0 has no previous")
# print("Original at index+1:", twists[index+1, 3:6] if index+1 < N-1 else "Index out of bounds for twists")

# Dirette differenze e stampa
# if index < vr.shape[0]:
#     direct_diff = vr[index] - twists[index, 3:6]
#     print("Direct difference at index:", direct_diff)
# print("##############################################")

n_valid = min(vr.shape[0], twists.shape[0])

# Stampa le differenze dirette
if method == 'pos':

    direct_diff_vr = vr[:n_valid] - positions[:n_valid]
    direct_diff_wr = wr[:n_valid] - orientations[:n_valid]
    print("Direct differences for vr:", direct_diff_vr[:5])
    print("Direct differences for wr:", direct_diff_wr[:5])

else:
     
    direct_diff_vr = vr[:n_valid] - twists[:n_valid, 3:6]
    direct_diff_wr = wr[:n_valid] - twists[:n_valid, 0:3]
    print("Direct differences for vr:", direct_diff_vr[:5])
    print("Direct differences for wr:", direct_diff_wr[:5])

# Calcolo degli errori di ricostruzione
errSP = errSP = np.hstack([(direct_diff_vr)**2, (direct_diff_wr)**2])

# Stampa i primi 5 errori per un'anteprima
print("Error squared matrix (errSP):", errSP[:5])
print("##############################################")

# # Calcolo dell'errore RMSE
# err = np.sum(errSP, axis=0) 
# print("Sum of squared errors (err):", err)

# Calcola la somma degli errori al quadrato per ogni gruppo
sum_squared_errors_1 = np.sum(errSP[:, :3])
sum_squared_errors_2 = np.sum(errSP[:, 3:6])

print("Sum of squared errors for group 1 (vr):", sum_squared_errors_1)
print("Sum of squared errors for group 2 (wr):", sum_squared_errors_2)
print("##############################################")
# # Calcolo finale della RMSE
# RMSE = np.sqrt([
#     np.sum(err[:3]),  # Somma degli errori per le componenti di vr
#     np.sum(err[3:])   # Somma degli errori per le componenti di wr
# ] / (N-3))

# # Stampa degli errori di ricostruzione (RMSE)
# print("Reconstruction errors (RMSE):", RMSE)

# Calcola RMSE per ciascun gruppo
RMSE_1 = np.sqrt(sum_squared_errors_1 / (N-3))
RMSE_2 = np.sqrt(sum_squared_errors_2 / (N-3))

print("RMSE 1:", RMSE_1)
print("RMSE 2:", RMSE_2)



if method == 'pos':
    # Plot original and reconstructed velocity
    plt.figure(figsize=(10, 6), num='DHB to Cartesian velocity')
    for i in range(6):
        plt.subplot(2, 3, i+1)
        if i < 3:
            plt.plot(T[:n_valid], orientations[:n_valid, i], 'g', linewidth=5, label='Original $\omega_{}$'.format(i+1))
            plt.plot(T[:n_valid], wr[:n_valid, i], 'b', linewidth=2, label='Reconstructed $\omega_{}$'.format(i+1))
            plt.ylabel('$\omega_{}$'.format(i+1))
        else:
            plt.plot(T[:n_valid], positions[:n_valid, i-3], 'g', linewidth=5, label='Original $x_{}$'.format(i-2))
            plt.plot(T[:n_valid], vr[:n_valid, i-3], 'b', linewidth=2, label='Reconstructed $x_{}$'.format(i-2))
            plt.ylabel('$x_{}$'.format(i-2))
        plt.grid(True)
        plt.legend()

    plt.tight_layout()

else:
    # Plot original and reconstructed velocity
    plt.figure(figsize=(10, 6), num='DHB to Cartesian velocity')
    for i in range(6):
        plt.subplot(2, 3, i+1)
        if i < 3:
            plt.plot(T[:n_valid], twists[:n_valid, i], 'g', linewidth=5, label='Original $\omega_{}$'.format(i+1))
            plt.plot(T[:n_valid], wr[:n_valid, i], 'b', linewidth=2, label='Reconstructed $\omega_{}$'.format(i+1))
            plt.ylabel('$\omega_{}$'.format(i+1))
        else:
            plt.plot(T[:n_valid], twists[:n_valid, i], 'g', linewidth=5, label='Original $x_{}$'.format(i-2))
            plt.plot(T[:n_valid], vr[:n_valid, i-3], 'b', linewidth=2, label='Reconstructed $x_{}$'.format(i-2))
            plt.ylabel('$x_{}$'.format(i-2))
        plt.grid(True)
        plt.legend()

    plt.tight_layout()




if method == 'pos':

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions[:,0], positions[:, 1], positions[:, 2], 'r-', label='Original positions')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(orientations[:,0], orientations[:, 1], orientations[:, 2], 'r-', label='Original orientations')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
        

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(vr[:, 0], vr[:, 1], vr[:, 2], 'b--', label='DHB Position.')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(wr[:, 0], wr[:, 1], wr[:, 2], 'b--', label='DHB Orientation')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

else:


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(twists[:, 3], twists[:, 4], twists[:, 5], 'r-', label='Original Linear Vel.')
    ax.plot(vr[:, 0], vr[:, 1], vr[:, 2], 'b--', label='DHB Linear Vel.')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()


    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(twists[:,0], twists[:, 1], twists[:, 2], 'r-', label='Original Angular Vel.')
    ax.plot(wr[:, 0], wr[:, 1], wr[:, 2], 'b--', label='DHB Angular Vel.')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()







plt.show()

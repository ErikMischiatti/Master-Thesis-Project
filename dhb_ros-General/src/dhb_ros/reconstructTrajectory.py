# Functions that reconstruct a Cartesian trajectory from its DHB invariant representation
# See: D. Lee, R. Soloperto, and M. Saveriano, "Bidirectional invariant
# representation of rigid body motions and its application to gesture
# recognition and reproduction", Auton. Robots, 42(1):125–145, 2018.

# Reconstruct Cartesian trajectory (position or velocity)
# Input: invariants -> DHB invariants ([N-2]x6 array) 
#        Hv0 -> Initial linear frame
#        Hw0 -> Initial angular frame
#
# Output: v -> position or linear velocity trajectory ([N-2]x3 array)
#         w -> relative rotation vector or angular velocity trajectory ([N-2]x3 array)

import numpy as np

def reconstructTrajectory(invariants, Hv0, Hw0, method):
    mv = invariants[:, 0]
    tv_1 = invariants[:, 1]
    tv_2 = invariants[:, 2]
    mw = invariants[:, 3]
    tw_1 = invariants[:, 4]
    tw_2 = invariants[:, 5]

    N = len(tv_1)

    v = np.zeros((N, 3))
    w = np.zeros((N, 3))

    coef = 1 if method == 'pos' else 0
    
    # Hv0 = np.array(Hv0)
    # Hw0 = np.array(Hw0)
    Hv0[3, 3] = coef

    for i in range(N):
        # Matrici di rotazione
        Rp = rotY(tv_1[i]) @ rotX(tv_2[i])
        Rr = rotY(tw_1[i]) @ rotX(tw_2[i])

        # Vettore di posizione o velocità
        P = np.array([[mv[i]], [0], [0]])

        # Trasformazione omogenea
        H = np.vstack((np.hstack((Rp, P)), [0, 0, 0, coef]))

        # # Debug: Stampa i valori intermedi
        # print(f"Iteration {i}:")
        # print(f"Rp:\n{Rp}")
        # print(f"Rr:\n{Rr}")
        # print(f"P:\n{P}")
        # print(f"H:\n{H}")
        # print(f"Hv0 before update:\n{Hv0}")

        # La ricostruzione della posizione è diversa dalla velocità
        if method == 'pos':
            v[i, :] = Hv0[0:3, 3]

        Hv0 = Hv0 @ H  # Update the linear frame

        # Debug: Verifica aggiornamento di Hv0
        # print(f"Hv0 after update:\n{Hv0}")

        if method != 'pos':
            v[i, :] = Hv0[0:3, 3]  

        # Velocità angolare o vettore di rotazione
        w[i, :] = (Hw0 @ np.array([mw[i], 0, 0])).T

        # Aggiorna il frame angolare
        Hw0 = Hw0 @ Rr

        # Debug: Verifica aggiornamento di Hw0
        # print(f"Hw0 after update:\n{Hw0}")

    return v, w

def rotX(phi):
    """Rotazione elementare attorno all'asse x."""
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def rotY(beta):
    """Rotazione elementare attorno all'asse y."""
    c, s = np.cos(beta), np.sin(beta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
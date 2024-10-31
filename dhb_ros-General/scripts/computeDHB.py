# Functions that implement the Denavit–Hartenberg inspired Bidirectional (DHB) invariant representation
# See: D. Lee, R. Soloperto, and M. Saveriano, "Bidirectional invariant
#      representation of rigid body motions and its application to gesture
#      recognition and reproduction", Auton. Robots, 42(1):125–145, 2018.
#
# Compute DHB invariants (position- or velocity-based)
# Input: p -> position difference or linear velocity (Nx3 array) 
#        theta -> rotation vector difference or angular velocity (Nx3 array)
#        method -> 'pos': poisiton-based DHB, 'vel': velocity-based DHB
#        pose0 -> initial pose (used only if method='pos')
# Output: m_p       -> first linear (position or velocity) invariant (N-2 array)
#         theta_p_1 -> second linear (position or velocity) invariant (N-2 array)
#         theta_p_2 -> third linear (position or velocity) invariant (N-2 array)
#
#         m_r       -> first angular (position or velocity) invariant (N-2 array)
#         theta_r_1 -> second angular (position or velocity) invariant (N-2 array)
#         theta_r_2 -> third angular (position or velocity) invariant (N-2 array)
#
#         H_p -> Initial linear frame
#         H_r -> Initial angular frame


import numpy as np

def computeDHB(p, theta, method, pose0=None):
    if method == 'pos' and pose0 is None:
        raise ValueError("pose0 must be provided for position-based DHB calculation.")
    loop_iter, _ = p.shape

    # Compute frames
    P_ex = c_ex(p[0], np.array([1, 0, 0]))
    P_ex2 = c_ex(p[1], P_ex)
    P_ey = c_ey(P_ex, P_ex2, (P_ex[1] - P_ex[2], P_ex[2] - P_ex[0], P_ex[0] - P_ex[1]) / np.linalg.norm((P_ex[1] - P_ex[2], P_ex[2] - P_ex[0], P_ex[0] - P_ex[1])))
    P_ez = np.cross(P_ex, P_ey)
    if np.linalg.norm(P_ez) > 1e-10:
        P_ez = P_ez / np.linalg.norm(P_ez)

    H_p = np.eye(4)
    H_p[:3, :3] = np.array([P_ex.T, P_ey.T, P_ez.T]).T
    if method == 'pos':
        H_p[:3, 3] = pose0
    else:
        H_p[:3, 3] = p[0]

    T_ex = c_ex(theta[0], np.array([1, 0, 0]))
    T_ex2 = c_ex(theta[1], T_ex)
    T_ey = c_ey(T_ex, T_ex2, np.array([0, 1, 0]))
    T_ez = np.cross(T_ex, T_ey)
    if np.linalg.norm(T_ez) > 1e-10:
        T_ez = T_ez / np.linalg.norm(T_ez)

    H_r = np.eye(3)
    H_r[:3, :3] = np.array([T_ex.T, T_ey.T, T_ez.T]).T

    m_p = np.zeros(loop_iter - 2)
    theta_p_1 = np.zeros(loop_iter - 2)
    theta_p_2 = np.zeros(loop_iter - 2)

    m_r = np.zeros(loop_iter - 2)
    theta_r_1 = np.zeros(loop_iter - 2)
    theta_r_2 = np.zeros(loop_iter - 2)

    # Compute invariant values
    for i in range(loop_iter - 2):
        P_ex3 = c_ex(p[i + 2], P_ex2)
        P_ey2 = c_ey(P_ex2, P_ex3, P_ey)
        # print(f"Loop {i}: P_ex2 shape: {P_ex2.shape}, P_ex3 shape: {P_ex3.shape}, P_ey shape: {P_ey.shape}, P_ey2 shape: {P_ey2.shape}")
        m_p[i], theta_p_1[i], theta_p_2[i] = c_values(p[i], P_ex, P_ex2, P_ey, P_ey2)

        P_ex = P_ex2
        P_ex2 = P_ex3
        P_ey = P_ey2

        T_ex3 = c_ex(theta[i + 2], T_ex2)
        T_ey2 = c_ey(T_ex2, T_ex3, T_ey)
        # print(f"Loop {i}: T_ex2 shape: {T_ex2.shape}, T_ex3 shape: {T_ex3.shape}, T_ey shape: {T_ey.shape}, T_ey2 shape: {T_ey2.shape}")
        m_r[i], theta_r_1[i], theta_r_2[i] = c_values(theta[i], T_ex, T_ex2, T_ey, T_ey2)

        T_ex = T_ex2
        T_ex2 = T_ex3
        T_ey = T_ey2

    invariants = np.column_stack((m_p, theta_p_1, theta_p_2, m_r, theta_r_1, theta_r_2))
    return invariants, H_p, H_r

def c_ex(u, epx):
    ex = u
    n_ex = np.linalg.norm(ex)
    if n_ex > 1e-10:
        ex = ex / n_ex
    else:
        ex = epx
    return ex

def c_ey(ex, ex2, eyp):
    if ex.shape != ex2.shape:
        raise ValueError(f"Shapes of ex and ex2 are not compatible: {ex.shape} and {ex2.shape}")
    ey = np.cross(ex, ex2)
    n_ey = np.linalg.norm(ey)
    if n_ey > 1e-10:
        ey = ey / n_ey
    else:
        ey = eyp
    return ey

def c_values(u, ex, ex2, ey, ey2):
    m = ex @ u.T
    theta1 = np.arctan2(np.cross(ex, ex2) @ ey.T, ex @ ex2.T)
    theta2 = np.arctan2(np.cross(ey, ey2) @ ex2.T, ey @ ey2.T)
    return m, theta1, theta2






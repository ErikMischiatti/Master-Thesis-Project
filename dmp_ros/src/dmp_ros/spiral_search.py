import numpy as np


def spiral_points(arc=1, separation=1):
    """generate points on an Archimedes' spiral
    with `arc` giving the length of arc between two points
    and `separation` giving the distance between consecutive
    turnings
    - approximate arc length with circle arc at given distance
    - use a spiral equation r = b * phi
    """

    def p2c(r, phi):
        """polar to cartesian
        """
        return (r * np.cos(phi), r * np.sin(phi))

    # yield a point at origin
    yield (0, 0)

    # initialize the next point in the required distance
    r = arc
    b = separation / (2 * np.pi)
    # find the first phi to satisfy distance of `arc` to the second point
    phi = float(r) / b
    while True:
        yield p2c(r, phi)
        # advance the variables
        # calculate phi that will give desired arc length at current radius
        # (approximating with circle)
        phi += float(arc) / r
        r = b * phi


def get_search_trajectory(start_pose, end_pose, press_force=2):
    """
    :param start_pose: x, y, z, qx, qy, qz, qw
    :param end_pose: x, y, z, qx, qy, qz, qw
    :param press_force: press force downwards in z-direction (in Newton)
    :return: ndarray of poses
    """

    # add spiral search traj ----------------------
    spiral_search_traj = spiral_points(arc=1, separation=1)
    max_spiral_points = 5000
    pos_spiral = np.zeros((max_spiral_points, 3))
    for i, p in enumerate(spiral_points(arc=0.00001, separation=1e-3)):
        if i >= max_spiral_points:
            break
        pos_spiral[i, :] = p + (0,)  # add z axis value

    # remove points where acceleration is too high, spiral start is delicate
    max_acceleration = 1e-6
    acc_spiral = np.diff(pos_spiral, n=2, axis=0)
    idx = np.argmax(np.linalg.norm(acc_spiral, axis=1) < max_acceleration)  # stops at first true

    pos_spiral = pos_spiral[idx:, :] - pos_spiral[idx, :]
    pos_spiral += start_pose[:3]

    spiral_num_samples = pos_spiral.shape[0]

    ori = np.tile(start_pose[3:], (spiral_num_samples, 1))
    wrench = np.zeros((spiral_num_samples, 6))
    wrench[:, 2] = press_force
    # gripper = np.tile([np.nan, np.nan],
    #                   (spiral_num_samples, 1))

    spiral_segment = np.hstack((pos_spiral, ori, wrench))

    return spiral_segment


if __name__ == "__main__":
    spiral_segment = get_search_trajectory([0, 0, 0, 0, 0, 0, 1], [0.1, 0.1, 0, 0, 0, 0, 1])

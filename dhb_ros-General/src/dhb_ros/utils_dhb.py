import importlib
import logging
import os
import subprocess

import numpy as np
import scipy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def set_logger_format(logger, level='DEBUG'):
    logging.root.setLevel(logging.NOTSET)
    ch = logging.StreamHandler()
    ch.setLevel(level)
    simple_format = "[%(name)s] [%(module)s.%(funcName)15.20s() ] [%(levelname)-7.7s]  %(message)s"
    formatter = logging.Formatter(simple_format)
    ch.setFormatter(formatter)
    logger.addHandler(ch)


def mahalanobis_distance(u, v, C, is_v_inverse=False):
    """
    The Mahalanobis distance between 1-D arrays `u` and `v` and Covariance matrix C
    """
    if is_v_inverse:
        CI = C
    else:
        CI = np.linalg.inv(C)
    return scipy.spatial.distance.mahalanobis(u, v, CI)


def clone_repository(repo_url, destination_folder=""):
    try:
        subprocess.check_output(['git', 'clone', repo_url, destination_folder], stderr=subprocess.STDOUT)
        print(f"Repository cloned successfully to {destination_folder}")
    except subprocess.CalledProcessError as e:
        print(f"Error cloning repository: {e.output.decode('utf-8')}")


def import_or_install(package):
    try:
        # Try to import the package
        importlib.import_module(package)
        print(f"{package} is already installed.")
    except ImportError:
        print(f"{package} is not installed. Installing...")
        install_package(package)


def install_package(package):
    import subprocess
    # subprocess.check_call(["pip", "install", "--user", package])
    subprocess.check_call(["pip", "install", "--user", package, "--index-url", "https://pypi.python.org/simple"])
    print(f"{package} has been installed.")


def create_directory(directory_path):
    if not os.path.exists(directory_path):
        try:
            os.makedirs(directory_path)
            print(f"Directory '{directory_path}' created successfully.")
        except OSError as e:
            print(f"Error creating directory '{directory_path}': {e}")
    else:
        print(f"Directory '{directory_path}' already exists.")


def get_franka_demo_time_for_algo_sample(sample, algo_divider=50, franka_state_rate=1000):
    sample_in_franka_state = sample * algo_divider
    time = sample_in_franka_state / franka_state_rate
    return time


def plot_3d_trajectory(data, time=None, show=True):
    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the trajectory
    ax.plot(data[:, 0], data[:, 1], data[:, 2], label='Trajectory')

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Trajectory')

    # Add a legend
    ax.legend()

    # Show plot
    if show:
        plt.show()


if __name__ == "__main__":
    get_franka_demo_time_for_algo_sample(110)

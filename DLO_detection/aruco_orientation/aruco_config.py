# aruco_config.py: 
# Configuration settings for ArUco marker detection, 
# including camera calibration data, marker sizes, and related constants.

import json
import os

def save_params(params, filename='/home/.../catkin_ws/src/DLO_detection/scripts/params_v2.json'):  # modify the path 
    """
    Save parameters to a JSON file.

    Args:
        params (dict): The parameters to be saved.
        filename (str): Path to the JSON file where parameters will be saved.
    """
    with open(filename, 'w') as f:
        json.dump(params, f, indent=4)  # indent improves readability

def load_params(filename='/home/.../catkin_ws/src/DLO_detection/scripts/params_v2.json'): # modify the path
    """
    Load parameters from a JSON file.

    Args:
        filename (str): Path to the JSON file from which to load parameters.

    Returns:
        dict or None: Loaded parameters or None if file not found.
    """
    if not os.path.exists(filename):
        print(f"Warning: File '{filename}' not found.")
        return None
    with open(filename, 'r') as f:
        return json.load(f)

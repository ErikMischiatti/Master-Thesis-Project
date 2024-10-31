# aruco_config.py: 
# Contains configuration settings for ArUco marker detection, 
# including camera calibration data, marker sizes, and other related constants.

import json

def save_params(params, filename='/home/asl_team/catkin_ws/src/DLO_detection/scripts/params_v2.json'):
    """Save parameters to a JSON file."""
    with open(filename, 'w') as f:
        json.dump(params, f)

def load_params(filename='/home/asl_team/catkin_ws/src/DLO_detection/scripts/params_v2.json'):
    """Load parameters from a JSON file."""
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        return None

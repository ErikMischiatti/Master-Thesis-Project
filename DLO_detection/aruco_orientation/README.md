# ArUco Orientation Module

This module contains scripts for detecting ArUco markers, processing images from Intel RealSense cameras, and tuning HSV parameters for object segmentation. Each script has specific responsibilities in the ArUco detection and visualization pipeline.

## Files Overview

### `aruco_config.py`
Contains configuration settings for ArUco marker detection, including camera calibration data, marker sizes, and other related constants. This script provides functions to save and load these parameters from a JSON file.

### `aruco_detection_filtered.py`
This script contains classes and functions to detect ArUco markers in a video stream, apply exponential smoothing to reduce noise in detected coordinates, and calculate rotation angles about the X and Z axes for each marker. The `ExponentialSmoothing` class stabilizes the coordinates, and the `ArucoDetector` class handles marker detection and rotation angle calculations.

### `aruco_realsense.py`
This script is dedicated to integrating Intel RealSense cameras into the ArUco detection pipeline. It handles the connection to the RealSense camera, retrieves depth data, and uses that information to enhance marker detection with 3D spatial information.

### `aruco_trackbars.py`
Implements a graphical user interface with trackbars to allow for real-time tuning of parameters such as thresholds or filter settings. This script enables fine control of parameters for optimizing the detection process.

### `aruco_vision.py`
Focuses on the computer vision aspects of the project, possibly containing functions for additional image processing, drawing, or visualizing the detected markers and their associated data. This includes drawing 3D axes and calculating rotation angles between connectors and cables.

### `hsv_tuner.py`
This script provides a graphical interface with trackbars to adjust HSV values for detecting connectors and cables using ArUco markers. It allows for real-time tuning of parameters, with the option to save them to a JSON file.

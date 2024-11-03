# DLO Detection

The `DLO_detection` ROS package is developed for detecting and locating ArUco markers, as well as segmenting connectors and cables based on HSV color filters. Integrated with ROS, the package uses an Intel RealSense camera to capture depth and RGB data, providing outputs of marker position and orientation in a 3D coordinate system.

## Directory Structure

### 1. **`scripts` Folder**
   Contains the main scripts for marker detection and image processing.

   - **`aruco_main.py`**: Main script that performs ArUco marker detection and segments connectors and cables using HSV filters. It publishes position and orientation data of markers and connectors as ROS messages, allowing integration with other ROS nodes.
   - **`params_v2.json`**: JSON file containing HSV parameters for segmenting connectors and cables in the image. These parameters are used by `aruco_main.py` to apply color filters and enhance detection accuracy.

### 2. **`launch` Folder**
   Contains launch files to start ROS nodes in a pre-configured environment.

   - **`camera_pose.launch`**: Configures and launches the camera position within the detection pipeline.
   - **`detection_pipeline.launch`**: Starts the complete detection pipeline by launching all necessary ROS nodes.
   - **`my_rviz_config.PANDA.yml`** and **`my_rviz_config.rviz`**: RViz configuration files to visualize ArUco markers, camera data, and other elements in the pipeline. These files are customized for a robotic arm model, like the PANDA.

### 3. **`aruco_orientation` Folder**
   Contains modules specifically for ArUco marker detection and orientation.

   - **`aruco_config.py`**: Defines and manages configuration settings such as camera calibration data and marker sizes.
   - **`aruco_detection_filtered.py`**: Detects ArUco markers and applies an exponential smoothing filter to stabilize the detected coordinates.
   - **`aruco_realsense.py`**: Integrates the RealSense camera to acquire depth data and enhance marker detection with 3D information.
   - **`aruco_trackbars.py`**: Provides a GUI with trackbars for real-time HSV parameter tuning.
   - **`aruco_vision.py`**: Manages image processing to draw and visualize detected ArUco markers.
   - **`hsv_tuner.py`**: Script to adjust HSV values for detecting connectors and cables.

### 4. **Main Files in the Directory**

   - **`CMakeLists.txt`**: CMake configuration file for building the package. It defines package dependencies and specifies where to install the Python scripts. Currently, the file is set up mainly for Python components but can be extended to include custom ROS messages, services, and actions if needed.
   - **`package.xml`**: ROS package definition file. Includes package name, version, maintainer details, and dependencies. Ensure that license and maintainer information is filled in.
   - **`requirements.txt`**: Lists the required Python libraries for the package, including `numpy`, `opencv-python`, and `pyrealsense2`. Specifies essential libraries for image processing and data acquisition from the RealSense camera.

## Setup

### Prerequisites

## Execution

## Notes



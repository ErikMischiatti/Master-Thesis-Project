# ArUco Detection and Pose Estimation

This module includes the main script `aruco_main.py` for detecting ArUco markers, locating connectors and cables in 3D space, and publishing the calculated pose and orientation information to ROS. Additionally, it uses a JSON file, `params_v2.json`, to store and load HSV parameters for accurate object segmentation.

## Files

### 1. `aruco_main.py`
- **Description**: The main script that initializes a ROS node to process image data from a RealSense camera, detect ArUco markers, and segment connector and cable regions. Key functions include:
  - **Marker Detection**: Uses the `ArucoDetector` class to identify ArUco markers in the camera feed and publish their 3D positions and orientations.
  - **Connector and Cable Segmentation**: Filters the image using HSV color masks defined in `params_v2.json` to locate and segment the connector and cable areas.
  - **Grasping Point Calculation**: Calculates a grasping point and orientation for a robotic arm, transforming the coordinates relative to the robot’s base.
  - **Smoothing and Transformations**: Applies exponential smoothing to rotation angles and manages transformations between camera and robot coordinate frames.
  - **ROS Publishing**: Publishes the detected pose and orientation information as ROS messages on specified topics for integration with other system components.

- **Usage**: 
  - Ensure that all dependencies, including ROS, OpenCV, `pyrealsense2`, and `pytransform3d`, are installed.
  - Run the script within a ROS environment. The node subscribes to image topics and publishes the processed pose and orientation data.

### 2. `params_v2.json`
- **Description**: This JSON file contains the HSV (Hue, Saturation, Value) parameters for the connector and cable segmentation. These values are used to apply color-based filters to the camera image, aiding in the identification of specific regions associated with the connector and cable.
- **Sample Structure**:
  ```json
  {
    "LH_Connector": 83, "LS_Connector": 91, "LV_Connector": 65,
    "UH_Connector": 112, "US_Connector": 255, "UV_Connector": 215,
    "LH_Cable": 0, "LS_Cable": 160, "LV_Cable": 161,
    "UH_Cable": 39, "US_Cable": 255, "UV_Cable": 255
  }

# aruco_detection_filtered:
# This script contains classes and functions to detect ArUco markers in a video stream,
# apply exponential smoothing to reduce noise in detected coordinates, and calculate 
# rotation angles about the X and Z axes for each marker. The `ExponentialSmoothing` 
# class stabilizes the coordinates, and the `ArucoDetector` class handles marker detection 
# and rotation angle calculations.

import pyrealsense2 as rs
import cv2
import numpy as np

class ExponentialSmoothing:
    def __init__(self, alpha=0.01):
        self.alpha = alpha  # Smoothing coefficient, 0 < alpha < 1
        self.smoothed_value = None

    def update(self, new_value):
        if self.smoothed_value is None:
            self.smoothed_value = new_value
        else:
            # Exponential smoothing
            self.smoothed_value = self.alpha * new_value + (1 - self.alpha) * self.smoothed_value
        return self.smoothed_value


class ArucoDetector:
    def __init__(self, marker_size):
        self.marker_size = marker_size
        self.smoother_x = ExponentialSmoothing(alpha=0.2)  # Per la coordinata X
        self.smoother_y = ExponentialSmoothing(alpha=0.2)  # Per la coordinata Y

    def detect_aruco_marker(self, frame):
        """Detects ArUco marker and returns rotation and translation vectors."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            offset_y = 30 
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, np.eye(3), np.zeros(4))

            for i in range(len(ids)):
                # Extract X and Y coordinates for the current marker
                x, y = tvecs[i][0][0], tvecs[i][0][1]

                # Apply exponential smoothing filter to X and Y coordinates
                smoothed_x = self.smoother_x.update(x)
                smoothed_y = self.smoother_y.update(y)
                
                cv2.putText(frame, f'ID: {ids[i][0]}, Smoothed X={smoothed_x:.2f}, Y={smoothed_y:.2f}', (10, offset_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                angle_x_deg , angle_z_deg = self.calculate_rotation_angles(rvecs[i])
                cv2.putText(frame, f'Rotation X: {angle_x_deg:.2f} deg', (10, offset_y + 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, f'Rotation Z: {angle_z_deg:.2f} deg', (10, offset_y + 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                offset_y += 60 

                # Draw the axes for each marker
                cv2.drawFrameAxes(frame, np.eye(3), np.zeros(4), rvecs[i], tvecs[i], 0.1)
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            return rvecs, tvecs

        return None, None

    def calculate_rotation_angles(self, rvec):
        """Calculate the rotation angles about the X and Z axes."""
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # Calculate rotation around the Z axis
        angle_z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        
        # Calculate rotation around the X axis
        angle_x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        
        angle_z_deg = np.degrees(angle_z)
        angle_x_deg = np.degrees(angle_x)
        
        return angle_x_deg, angle_z_deg

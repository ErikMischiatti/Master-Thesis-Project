import pyrealsense2 as rs
import cv2
import numpy as np

class ArucoDetector:
    def __init__(self, marker_size):
        self.marker_size = marker_size

    def detect_aruco_marker(self, frame):
        """Detects ArUco marker and returns rotation and translation vectors."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Define the dictionary and parameters
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        aruco_params = cv2.aruco.DetectorParameters()

        # Create the detector
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # Detect the markers
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            # Estimate pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, np.eye(3), np.zeros(4))
            
            for i in range(len(ids)):
                # Draw the axis for each marker
                cv2.drawFrameAxes(frame, np.eye(3), np.zeros(4), rvecs[i], tvecs[i], 0.1)
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                # Calculate rotation angle about the primary axis
                angle_deg = self.calculate_rotation_about_axis(rvecs[i])
                cv2.putText(frame, f'Rotation about axis: {angle_deg:.2f} deg', (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            return rvecs, tvecs

        return None, None

    def calculate_rotation_about_axis(self, rvec):
        """Calculate rotation around the axis using rotation vectors."""
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        angle_rad = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        angle_deg = np.degrees(angle_rad)
        return angle_deg
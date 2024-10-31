import pyrealsense2 as rs
import cv2
import numpy as np

class ExponentialSmoothing:
    def __init__(self, alpha=0.01):
        self.alpha = alpha  # Coefficiente di smorzamento, 0 < alpha < 1
        self.smoothed_value = None

    def update(self, new_value):
        if self.smoothed_value is None:
            # All'inizio, usa il primo valore come valore smussato
            self.smoothed_value = new_value
        else:
            # Formula del filtraggio esponenziale
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

        # Define the dictionary and parameters
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        aruco_params = cv2.aruco.DetectorParameters()

        # Create the detector
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # Detect the markers
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            offset_y = 30  # Inizializza l'offset verticale
            # Estimate pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, np.eye(3), np.zeros(4))

            for i in range(len(ids)):
                # Extract X and Y coordinates for the current marker
                x, y = tvecs[i][0][0], tvecs[i][0][1]

                # Apply exponential smoothing filter to X and Y coordinates
                smoothed_x = self.smoother_x.update(x)
                smoothed_y = self.smoother_y.update(y)

                # import pdb
                # pdb.set_trace()
                # Display the smoothed coordinates on the frame con offset_y
                cv2.putText(frame, f'ID: {ids[i][0]}, Smoothed X={smoothed_x:.2f}, Y={smoothed_y:.2f}', (10, offset_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Calculate rotation angle about X and Z axis 
                angle_x_deg , angle_z_deg = self.calculate_rotation_angles(rvecs[i])
                
                # Display the rotation angles with offset_y
                cv2.putText(frame, f'Rotation X: {angle_x_deg:.2f} deg', (10, offset_y + 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, f'Rotation Z: {angle_z_deg:.2f} deg', (10, offset_y + 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Incrementa l'offset_y per evitare sovrapposizioni
                offset_y += 60  # Aumenta l'offset di 60 pixel per spostare le scritte successive

                # Draw the axes for each marker
                cv2.drawFrameAxes(frame, np.eye(3), np.zeros(4), rvecs[i], tvecs[i], 0.1)
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            return rvecs, tvecs

        return None, None

    # def calculate_rotation_about_axis(self, rvec):
    #     """Calculate the rotation angle about the primary axis (e.g., Z-axis)."""
    #     rotation_matrix, _ = cv2.Rodrigues(rvec)
    #     # Extract the angle from the rotation matrix (this is a simplification for Z-axis rotation)
    #     angle = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    #     return np.degrees(angle)
    


    def calculate_rotation_angles(self, rvec):
        """Calculate the rotation angles about the X and Z axes."""
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # Calcolo della rotazione attorno all'asse Z
        angle_z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        
        # Calcolo della rotazione attorno all'asse X
        angle_x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        
        # Converti gli angoli in gradi
        angle_z_deg = np.degrees(angle_z)
        angle_x_deg = np.degrees(angle_x)
        
        return angle_x_deg, angle_z_deg
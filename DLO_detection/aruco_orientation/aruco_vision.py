# aruco_vision.py: 
# Focuses on the computer vision aspects of the project, 
# possibly containing functions for additional image processing, drawing, or visualizing 
# the detected markers and their associated data.

import cv2
import rospy
import numpy as np
import math
import pyrealsense2 as rs

def find_connector_axis_3d(connector_centers_3d):
    """
    Trova l'asse 3D che collega due punti distinti del connettore.
    connector_centers_3d: lista di tuple che rappresentano i centri 3D delle macro aree del connettore.
    """
    if len(connector_centers_3d) < 2:
        return None, None

    # Prendi i primi due punti trovati
    point1 = connector_centers_3d[0]
    point2 = connector_centers_3d[1]


    return point1, point2

def draw_3d_axis_on_frame(frame, point1, point2, camera_matrix):
    """
    Disegna l'asse 3D sul frame.
    """
    if point1 is not None and point2 is not None:
        # Proietta i punti 3D nel piano immagine
        # start_pixel = rs.rs2_project_point_to_pixel(depth_intrin, list(point1))
        # end_pixel = rs.rs2_project_point_to_pixel(depth_intrin, list(point2))
        # import pdb
        # pdb.set_trace()
        # print(point1)
        # print(point2)
        start_pixel, _ =  cv2.projectPoints(point1[np.newaxis, ...], np.array([[0.0, 0.0, 0.0]]), np.array([[0.0, 0.0, 0.0]]), camera_matrix, np.array([0.0, 0.0, 0.0, 0.0, 0.0]))
        end_pixel, _ =  cv2.projectPoints(point2[np.newaxis, ...], np.array([[0.0, 0.0, 0.0]]), np.array([[0.0, 0.0, 0.0]]), camera_matrix, np.array([0.0, 0.0, 0.0, 0.0, 0.0]))

        # Log di debug per i pixel proiettati
        # rospy.loginfo(f"Start pixel: {start_pixel}, End pixel: {end_pixel}")

        # Verifica la presenza di NaN nei punti proiettati
        # if not any(math.isnan(coord) for coord in start_pixel + end_pixel):
            # Disegna la linea sull'immagine
        # import pdb
        # pdb.set_trace()
        cv2.line(frame, (int(start_pixel[0,0,0]), int(start_pixel[0,0,1])), (int(end_pixel[0,0,0]), int(end_pixel[0,0,1])), (0, 255, 255), 2)
        cv2.putText(frame, 'Axis', (int((start_pixel[0,0,0] + end_pixel[0,0,0]) / 2), int((start_pixel[0,0,1] + end_pixel[0,0,1]) / 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)
            
# def calculate_rotation_angle(point1, point2):
#     """
#     Calcola l'angolo di rotazione dell'asse del connettore rispetto all'asse Z nel piano XY.
#     Restituisce l'angolo in gradi compreso tra 0° e 360°.
#     """
#     # Vettore tra i due punti
#     vector = np.array([point2[0] - point1[0], point2[1] - point1[1]])

#     # Calcola l'angolo rispetto all'asse X nel piano XY
#     angle_radians = math.atan2(vector[1], vector[0])  # y, x per il piano XY
#     angle_degrees = math.degrees(angle_radians)

#     # Normalizzazione per avere un angolo tra 0° e 360°
#     if angle_degrees < 0:
#         angle_degrees += 360
#     return angle_degrees

class ExponentialSmoothing:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.smoothed_angle = None

    def update(self, new_angle):
        if self.smoothed_angle is None:
            self.smoothed_angle = new_angle
        else:
            self.smoothed_angle = self.alpha * new_angle + (1 - self.alpha) * self.smoothed_angle
        return self.smoothed_angle

# Inizializza il filtro per l'angolo
angle_smoother = ExponentialSmoothing(alpha=0.1)

def calculate_rotation_angle(point1, point2):
    """
    Calcola l'angolo di rotazione dell'asse del connettore rispetto all'asse Z nel piano XY
    e applica un filtro di smoothing.
    """
    vector = np.array([point2[0] - point1[0], point2[1] - point1[1]])
    angle_radians = math.atan2(vector[1], vector[0])  # Calcola l'angolo rispetto all'asse X nel piano XY
    angle_degrees = math.degrees(angle_radians)

    if angle_degrees < 0:
        angle_degrees += 360

    # Applica lo smoothing
    smoothed_angle = angle_smoother.update(angle_degrees)
    
    return smoothed_angle

        
def publish_pose(pub, X, Y, Z):
    """Publish 3D position as a ROS Pose message."""
    from geometry_msgs.msg import Pose, Point, Quaternion
    pose = Pose()
    pose.position = Point(X, Y, Z)
    pose.orientation = Quaternion(0, 0, 0, 1)  
    pub.publish(pose)


def segment_cable(cable_contour, depth_frame, depth_intrin, connector_center, max_distance_cm=5, segment_length=0.01):
    """Segment the cable using contours and depth information."""
    segmented_points = []
    length = len(cable_contour)
    step = max(1, int(length * segment_length))  # Ensure step is not zero
    for i in range(0, length, step):
        x, y = cable_contour[i][0]
        depth = depth_frame.get_distance(x, y)
        if depth > 0:
            X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)
            if not (math.isnan(X) or math.isnan(Y) or math.isnan(Z)):
                distance = math.sqrt((X - connector_center[0])**2 + (Y - connector_center[1])**2 + (Z - connector_center[2])**2)
                if distance <= max_distance_cm / 100.0:  # Convert cm to meters
                    segmented_points.append((X, Y, Z))
    return segmented_points

def process_connector_and_cable_axis(self, connector_center_3d, cable_center_3d, depth_intrin, frame):
    """
    Calcola e disegna l'asse 3D tra il connettore e il cavo.
    """
    if connector_center_3d is None or cable_center_3d is None:
        return

    # Disegna l'asse 3D tra connettore e cavo
    draw_3d_axis_on_frame(frame, connector_center_3d, cable_center_3d, depth_intrin)

    # Calcola l'angolo di rotazione dell'asse rispetto all'asse X
    angle = calculate_rotation_angle(connector_center_3d, cable_center_3d)
    cv2.putText(frame, f'Connector-Cable Angle: {angle:.2f} degrees', (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Pubblica l'asse come Pose su ROS
    self.publish_axis(connector_center_3d, cable_center_3d, angle)

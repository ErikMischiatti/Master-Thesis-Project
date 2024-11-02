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
    Finds the 3D axis connecting two distinct points of the connector.
    connector_centers_3d: list of tuples representing the 3D centers of the connector's macro areas.
    """
    if len(connector_centers_3d) < 2:
        return None, None

    # Take the first 2 points found 
    point1 = connector_centers_3d[0]
    point2 = connector_centers_3d[1]

    return point1, point2

def draw_3d_axis_on_frame(frame, point1, point2, camera_matrix):
    """
    Draws the 3D axis on the frame.
    """
    if point1 is not None and point2 is not None:
        # Project 3D points onto the image plane
        start_pixel, _ =  cv2.projectPoints(point1[np.newaxis, ...], np.array([[0.0, 0.0, 0.0]]), np.array([[0.0, 0.0, 0.0]]), camera_matrix, np.array([0.0, 0.0, 0.0, 0.0, 0.0]))
        end_pixel, _ =  cv2.projectPoints(point2[np.newaxis, ...], np.array([[0.0, 0.0, 0.0]]), np.array([[0.0, 0.0, 0.0]]), camera_matrix, np.array([0.0, 0.0, 0.0, 0.0, 0.0]))

        # Draw the line on the image
        cv2.line(frame, (int(start_pixel[0,0,0]), int(start_pixel[0,0,1])), (int(end_pixel[0,0,0]), int(end_pixel[0,0,1])), (0, 255, 255), 2)
        cv2.putText(frame, 'Axis', (int((start_pixel[0,0,0] + end_pixel[0,0,0]) / 2), int((start_pixel[0,0,1] + end_pixel[0,0,1]) / 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)
            

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

angle_smoother = ExponentialSmoothing(alpha=0.1)

def calculate_rotation_angle(point1, point2):
    """
    Calculates the rotation angle of the connector's axis relative to the Z-axis in the XY plane
    and applies a smoothing filter.
    """
    vector = np.array([point2[0] - point1[0], point2[1] - point1[1]])
    angle_radians = math.atan2(vector[1], vector[0])
    angle_degrees = math.degrees(angle_radians)

    if angle_degrees < 0:
        angle_degrees += 360
        
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
    step = max(1, int(length * segment_length)) 
    for i in range(0, length, step):
        x, y = cable_contour[i][0]
        depth = depth_frame.get_distance(x, y)
        if depth > 0:
            X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)
            if not (math.isnan(X) or math.isnan(Y) or math.isnan(Z)):
                distance = math.sqrt((X - connector_center[0])**2 + (Y - connector_center[1])**2 + (Z - connector_center[2])**2)
                if distance <= max_distance_cm / 100.0:
                    segmented_points.append((X, Y, Z))
    return segmented_points

def process_connector_and_cable_axis(self, connector_center_3d, cable_center_3d, depth_intrin, frame):
    """
    Calculates and draws the 3D axis between the connector and cable.
    """
    if connector_center_3d is None or cable_center_3d is None:
        return

    draw_3d_axis_on_frame(frame, connector_center_3d, cable_center_3d, depth_intrin)
    
    angle = calculate_rotation_angle(connector_center_3d, cable_center_3d)
    cv2.putText(frame, f'Connector-Cable Angle: {angle:.2f} degrees', (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Publish the axis as a Pose in ROS
    self.publish_axis(connector_center_3d, cable_center_3d, angle)

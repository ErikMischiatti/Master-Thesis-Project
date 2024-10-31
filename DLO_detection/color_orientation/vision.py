import cv2
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

def draw_3d_axis_on_frame(frame, point1, point2, depth_intrin):
    """
    Disegna l'asse 3D sul frame.
    """
    if point1 is not None and point2 is not None:
        # Proietta i punti 3D nel piano immagine
        start_pixel = rs.rs2_project_point_to_pixel(depth_intrin, list(point1))
        end_pixel = rs.rs2_project_point_to_pixel(depth_intrin, list(point2))

        # Verifica la presenza di NaN nei punti proiettati
        if not any(math.isnan(coord) for coord in start_pixel + end_pixel):
            # Disegna la linea sull'immagine
            cv2.line(frame, (int(start_pixel[0]), int(start_pixel[1])), (int(end_pixel[0]), int(end_pixel[1])), (0, 255, 255), 2)
            cv2.putText(frame, 'Axis', (int((start_pixel[0] + end_pixel[0]) / 2), int((start_pixel[1] + end_pixel[1]) / 2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)
            
def calculate_rotation_angle(point1, point2):
    """
    Calcola l'angolo di rotazione dell'asse del connettore rispetto all'asse X nel piano XY.
    Restituisce l'angolo in gradi compreso tra 0° e 360°.
    """
    # Vettore tra i due punti
    vector = np.array(point2) - np.array(point1)

    # Calcola l'angolo rispetto all'asse X
    angle_radians = math.atan2(vector[1], vector[0])
    angle_degrees = math.degrees(angle_radians)

    # Normalization
    if angle_degrees < 0:
        angle_degrees += 360
    return angle_degrees
        
def publish_pose(pub, X, Y, Z):
    """Publish 3D position as a ROS Pose message."""
    from geometry_msgs.msg import Pose, Point, Quaternion
    pose = Pose()
    pose.position = Point(X, Y, Z)
    pose.orientation = Quaternion(0, 0, 0, 1)  
    pub.publish(pose)

def detect_colored_sides(hsv, values, frame, depth_frame):
    """Detect and highlight colored sides of the connector."""
    sides = ['top', 'bottom', 'left', 'right']
    colors = {
        'top': (255, 0, 0),    
        'bottom': (0, 255, 0),
        'left': (0, 0, 255),    
        'right': (255, 255, 0)  
    }
    points_3d = {}

    for side in sides:
        lower = np.array([values[f'l_h_{side}'], values[f'l_s_{side}'], values[f'l_v_{side}']])
        upper = np.array([values[f'u_h_{side}'], values[f'u_s_{side}'], values[f'u_v_{side}']])
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 50:
                x, y, w, h = cv2.boundingRect(contour)
                center = (x + w // 2, y + h // 2)

                cv2.drawContours(frame, [contour], 0, colors[side], 2)
                cv2.circle(frame, center, 5, colors[side], -1)

                u, v = center
                depth = depth_frame.get_distance(u, v)
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [u, v], depth)

                if not math.isnan(X) and not math.isnan(Y) and not math.isnan(Z):
                    points_3d[side] = (X, Y, Z)
                    cv2.putText(frame, f'{side.capitalize()} X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}', (u + 10, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[side], 2, cv2.LINE_AA)

    return points_3d

def visualize_side_masks(hsv, values):
    """Create a combined mask for all connector sides."""
    sides = ['top', 'bottom', 'left', 'right']
    combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for side in sides:
        lower = np.array([values[f'l_h_{side}'], values[f'l_s_{side}'], values[f'l_v_{side}']])
        upper = np.array([values[f'u_h_{side}'], values[f'u_s_{side}'], values[f'u_v_{side}']])
        mask = cv2.inRange(hsv, lower, upper)
        combined_mask = cv2.bitwise_or(combined_mask, mask)

    return combined_mask

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
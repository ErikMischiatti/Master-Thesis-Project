#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import json
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge, CvBridgeError
import tf
import pyrealsense2 as rs

def nothing(x):
    pass

def save_params(params, filename='params.json'):
    with open(filename, 'w') as f:
        json.dump(params, f)

def load_params(filename='params.json'):
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        return None

def setup_trackbars():
    cv2.namedWindow('Parameters', cv2.WINDOW_NORMAL)
    # Trackbars for connector
    cv2.createTrackbar('LH Connector', 'Parameters', 0, 179, nothing)
    cv2.createTrackbar('LS Connector', 'Parameters', 0, 255, nothing)
    cv2.createTrackbar('LV Connector', 'Parameters', 0, 255, nothing)
    cv2.createTrackbar('UH Connector', 'Parameters', 179, 179, nothing)
    cv2.createTrackbar('US Connector', 'Parameters', 255, 255, nothing)
    cv2.createTrackbar('UV Connector', 'Parameters', 255, 255, nothing)
    # Trackbars for cable
    cv2.createTrackbar('LH Cable', 'Parameters', 0, 179, nothing)
    cv2.createTrackbar('LS Cable', 'Parameters', 0, 255, nothing)
    cv2.createTrackbar('LV Cable', 'Parameters', 0, 255, nothing)
    cv2.createTrackbar('UH Cable', 'Parameters', 179, 179, nothing)
    cv2.createTrackbar('US Cable', 'Parameters', 255, 255, nothing)
    cv2.createTrackbar('UV Cable', 'Parameters', 255, 255, nothing)
    # Trackbars for top side of connector
    cv2.createTrackbar('LH Top', 'Parameters', 0, 179, nothing)
    cv2.createTrackbar('LS Top', 'Parameters', 0, 255, nothing)
    cv2.createTrackbar('LV Top', 'Parameters', 0, 255, nothing)
    cv2.createTrackbar('UH Top', 'Parameters', 179, 179, nothing)
    cv2.createTrackbar('US Top', 'Parameters', 255, 255, nothing)
    cv2.createTrackbar('UV Top', 'Parameters', 255, 255, nothing)
    # Trackbars for bottom side of connector
    cv2.createTrackbar('LH Bottom', 'Parameters', 0, 179, nothing)
    cv2.createTrackbar('LS Bottom', 'Parameters', 0, 255, nothing)
    cv2.createTrackbar('LV Bottom', 'Parameters', 0, 255, nothing)
    cv2.createTrackbar('UH Bottom', 'Parameters', 179, 179, nothing)
    cv2.createTrackbar('US Bottom', 'Parameters', 255, 255, nothing)
    cv2.createTrackbar('UV Bottom', 'Parameters', 255, 255, nothing)
    # Trackbars for left side of connector
    cv2.createTrackbar('LH Left', 'Parameters', 0, 179, nothing)
    cv2.createTrackbar('LS Left', 'Parameters', 0, 255, nothing)
    cv2.createTrackbar('LV Left', 'Parameters', 0, 255, nothing)
    cv2.createTrackbar('UH Left', 'Parameters', 179, 179, nothing)
    cv2.createTrackbar('US Left', 'Parameters', 255, 255, nothing)
    cv2.createTrackbar('UV Left', 'Parameters', 255, 255, nothing)
    # Trackbars for right side of connector
    cv2.createTrackbar('LH Right', 'Parameters', 0, 179, nothing)
    cv2.createTrackbar('LS Right', 'Parameters', 0, 255, nothing)
    cv2.createTrackbar('LV Right', 'Parameters', 0, 255, nothing)
    cv2.createTrackbar('UH Right', 'Parameters', 179, 179, nothing)
    cv2.createTrackbar('US Right', 'Parameters', 255, 255, nothing)
    cv2.createTrackbar('UV Right', 'Parameters', 255, 255, nothing)

def load_saved_params():
    params = load_params()
    if params:
        # Load connector params
        cv2.setTrackbarPos('LH Connector', 'Parameters', params['LH_Connector'])
        cv2.setTrackbarPos('LS Connector', 'Parameters', params['LS_Connector'])
        cv2.setTrackbarPos('LV Connector', 'Parameters', params['LV_Connector'])
        cv2.setTrackbarPos('UH Connector', 'Parameters', params['UH_Connector'])
        cv2.setTrackbarPos('US Connector', 'Parameters', params['US_Connector'])
        cv2.setTrackbarPos('UV Connector', 'Parameters', params['UV_Connector'])
        # Load cable params
        cv2.setTrackbarPos('LH Cable', 'Parameters', params['LH_Cable'])
        cv2.setTrackbarPos('LS Cable', 'Parameters', params['LS_Cable'])
        cv2.setTrackbarPos('LV Cable', 'Parameters', params['LV_Cable'])
        cv2.setTrackbarPos('UH Cable', 'Parameters', params['UH_Cable'])
        cv2.setTrackbarPos('US Cable', 'Parameters', params['US_Cable'])
        cv2.setTrackbarPos('UV Cable', 'Parameters', params['UV_Cable'])
        # Load side colors params
        cv2.setTrackbarPos('LH Top', 'Parameters', params['LH_Top'])
        cv2.setTrackbarPos('LS Top', 'Parameters', params['LS_Top'])
        cv2.setTrackbarPos('LV Top', 'Parameters', params['LV_Top'])
        cv2.setTrackbarPos('UH Top', 'Parameters', params['UH_Top'])
        cv2.setTrackbarPos('US Top', 'Parameters', params['US_Top'])
        cv2.setTrackbarPos('UV Top', 'Parameters', params['UV_Top'])
        cv2.setTrackbarPos('LH Bottom', 'Parameters', params['LH_Bottom'])
        cv2.setTrackbarPos('LS Bottom', 'Parameters', params['LS_Bottom'])
        cv2.setTrackbarPos('LV Bottom', 'Parameters', params['LV_Bottom'])
        cv2.setTrackbarPos('UH Bottom', 'Parameters', params['UH_Bottom'])
        cv2.setTrackbarPos('US Bottom', 'Parameters', params['US_Bottom'])
        cv2.setTrackbarPos('UV Bottom', 'Parameters', params['UV_Bottom'])
        cv2.setTrackbarPos('LH Left', 'Parameters', params['LH_Left'])
        cv2.setTrackbarPos('LS Left', 'Parameters', params['LS_Left'])
        cv2.setTrackbarPos('LV Left', 'Parameters', params['LV_Left'])
        cv2.setTrackbarPos('UH Left', 'Parameters', params['UH_Left'])
        cv2.setTrackbarPos('US Left', 'Parameters', params['US_Left'])
        cv2.setTrackbarPos('UV Left', 'Parameters', params['UV_Left'])
        cv2.setTrackbarPos('LH Right', 'Parameters', params['LH_Right'])
        cv2.setTrackbarPos('LS Right', 'Parameters', params['LS_Right'])
        cv2.setTrackbarPos('LV Right', 'Parameters', params['LV_Right'])
        cv2.setTrackbarPos('UH Right', 'Parameters', params['UH_Right'])
        cv2.setTrackbarPos('US Right', 'Parameters', params['US_Right'])
        cv2.setTrackbarPos('UV Right', 'Parameters', params['UV_Right'])

def get_trackbar_values():
    values = {
        'l_h_connector': cv2.getTrackbarPos('LH Connector', 'Parameters'),
        'l_s_connector': cv2.getTrackbarPos('LS Connector', 'Parameters'),
        'l_v_connector': cv2.getTrackbarPos('LV Connector', 'Parameters'),
        'u_h_connector': cv2.getTrackbarPos('UH Connector', 'Parameters'),
        'u_s_connector': cv2.getTrackbarPos('US Connector', 'Parameters'),
        'u_v_connector': cv2.getTrackbarPos('UV Connector', 'Parameters'),
        'l_h_cable': cv2.getTrackbarPos('LH Cable', 'Parameters'),
        'l_s_cable': cv2.getTrackbarPos('LS Cable', 'Parameters'),
        'l_v_cable': cv2.getTrackbarPos('LV Cable', 'Parameters'),
        'u_h_cable': cv2.getTrackbarPos('UH Cable', 'Parameters'),
        'u_s_cable': cv2.getTrackbarPos('US Cable', 'Parameters'),
        'u_v_cable': cv2.getTrackbarPos('UV Cable', 'Parameters'),
        'l_h_top': cv2.getTrackbarPos('LH Top', 'Parameters'),
        'l_s_top': cv2.getTrackbarPos('LS Top', 'Parameters'),
        'l_v_top': cv2.getTrackbarPos('LV Top', 'Parameters'),
        'u_h_top': cv2.getTrackbarPos('UH Top', 'Parameters'),
        'u_s_top': cv2.getTrackbarPos('US Top', 'Parameters'),
        'u_v_top': cv2.getTrackbarPos('UV Top', 'Parameters'),
        'l_h_bottom': cv2.getTrackbarPos('LH Bottom', 'Parameters'),
        'l_s_bottom': cv2.getTrackbarPos('LS Bottom', 'Parameters'),
        'l_v_bottom': cv2.getTrackbarPos('LV Bottom', 'Parameters'),
        'u_h_bottom': cv2.getTrackbarPos('UH Bottom', 'Parameters'),
        'u_s_bottom': cv2.getTrackbarPos('US Bottom', 'Parameters'),
        'u_v_bottom': cv2.getTrackbarPos('UV Bottom', 'Parameters'),
        'l_h_left': cv2.getTrackbarPos('LH Left', 'Parameters'),
        'l_s_left': cv2.getTrackbarPos('LS Left', 'Parameters'),
        'l_v_left': cv2.getTrackbarPos('LV Left', 'Parameters'),
        'u_h_left': cv2.getTrackbarPos('UH Left', 'Parameters'),
        'u_s_left': cv2.getTrackbarPos('US Left', 'Parameters'),
        'u_v_left': cv2.getTrackbarPos('UV Left', 'Parameters'),
        'l_h_right': cv2.getTrackbarPos('LH Right', 'Parameters'),
        'l_s_right': cv2.getTrackbarPos('LS Right', 'Parameters'),
        'l_v_right': cv2.getTrackbarPos('LV Right', 'Parameters'),
        'u_h_right': cv2.getTrackbarPos('UH Right', 'Parameters'),
        'u_s_right': cv2.getTrackbarPos('US Right', 'Parameters'),
        'u_v_right': cv2.getTrackbarPos('UV Right', 'Parameters'),
    }
    return values

def publish_pose(pub, X, Y, Z):
    pose = Pose()
    pose.position = Point(X, Y, Z)
    pose.orientation = Quaternion(0, 0, 0, 1)  
    pub.publish(pose)

def segment_cable(cable_contour, depth_frame, depth_intrin, connector_center, max_distance_cm=5, segment_length=0.01):
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

def detect_colored_sides(hsv, values, frame, depth_frame):
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
    sides = ['top', 'bottom', 'left', 'right']
    combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for side in sides:
        lower = np.array([values[f'l_h_{side}'], values[f'l_s_{side}'], values[f'l_v_{side}']])
        upper = np.array([values[f'u_h_{side}'], values[f'u_s_{side}'], values[f'u_v_{side}']])
        mask = cv2.inRange(hsv, lower, upper)
        combined_mask = cv2.bitwise_or(combined_mask, mask)

    return combined_mask

class ImageDepthProcessor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('image_depth_processor', anonymous=True)

        # Bridge for converting between ROS Image and OpenCV
        self.bridge = CvBridge()

        # Publisher for 3D pose
        self.pose_pub = rospy.Publisher('cable_pose', Pose, queue_size=10)

        # Synchronize RGB and Depth frames
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.pipeline.start(self.config)

        # Setup trackbars and load previous parameters
        setup_trackbars()
        load_saved_params()

    def process_frames(self):
        align_to = rs.stream.color
        align = rs.align(align_to)

        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Ottieni le intrinseche del frame di profondità
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            frame = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            frame_height, frame_width = frame.shape[:2]
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            values = get_trackbar_values()

            # Rilevamento dei quattro lati del connettore
            points_3d = detect_colored_sides(hsv, values, frame, depth_frame)

            # Visualizzazione della maschera combinata dei lati
            combined_side_mask = visualize_side_masks(hsv, values)
            cv2.imshow('Combined Side Mask', combined_side_mask)

            # Mask for connector
            lower_connector = np.array([values['l_h_connector'], values['l_s_connector'], values['l_v_connector']])
            upper_connector = np.array([values['u_h_connector'], values['u_s_connector'], values['u_v_connector']])
            mask_connector = cv2.inRange(hsv, lower_connector, upper_connector)

            contours_connector, _ = cv2.findContours(mask_connector, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            connector_centers_3d = []

            # Process each significant contour
            for contour in contours_connector:
                if cv2.contourArea(contour) > 100:  # Filter out small areas
                    x, y, w, h = cv2.boundingRect(contour)
                    center = (x + w // 2, y + h // 2)

                    cv2.drawContours(frame, [contour], 0, (0, 255, 0), 2)
                    cv2.circle(frame, center, 5, (0, 255, 0), -1)

                    u, v = center
                    depth = depth_frame.get_distance(u, v)
                    
                    # Utilizza depth_intrin qui
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [u, v], depth)
                    center_3d = (X, Y, Z)

                    if not math.isnan(X) and not math.isnan(Y) and not math.isnan(Z):
                        connector_centers_3d.append(center_3d)
                        # rospy.loginfo(f'Connector 3D Coordinates: X={X}, Y={Y}, Z={Z}')
                        cv2.putText(frame, f'Connector X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}', (u + 10, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

            # Optionally publish the first detected connector's position
            if connector_centers_3d:
                publish_pose(self.pose_pub, *connector_centers_3d[0])

            # Mask for cable
            lower_cable = np.array([values['l_h_cable'], values['l_s_cable'], values['l_v_cable']])
            upper_cable = np.array([values['u_h_cable'], values['u_s_cable'], values['u_v_cable']])
            mask_cable = cv2.inRange(hsv, lower_cable, upper_cable)

            contours_cable, _ = cv2.findContours(mask_cable, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_cable:
                cable_contour = max(contours_cable, key=cv2.contourArea)

                # Define ROI around the connector
                x, y, w, h = cv2.boundingRect(cable_contour)
                roi = (x - 6 * w, y - 6 * h, x + 12 * w, y + 12 * h)

                # Filter cable contours within the ROI
                filtered_cable_contour = [pt for pt in cable_contour if roi[0] <= pt[0][0] <= roi[2] and roi[1] <= pt[0][1] <= roi[3]]
                filtered_cable_contour = np.array(filtered_cable_contour).reshape((-1, 1, 2))

                if len(filtered_cable_contour) > 0:
                    cv2.drawContours(frame, [filtered_cable_contour], 0, (255, 0, 0), 2)

                    # Calculate the center of the cable contour
                    x, y, w, h = cv2.boundingRect(filtered_cable_contour)
                    cable_center = (x + w // 2, y + h // 2)
                    cv2.circle(frame, cable_center, 5, (0, 0, 255), -1)  # Different color for cable center

                    u, v = cable_center
                    depth = depth_frame.get_distance(u, v)
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [u, v], depth)
                    cable_center_3d = (X, Y, Z)

                    if not math.isnan(X) and not math.isnan(Y) and not math.isnan(Z):
                        # rospy.loginfo(f'Cable 3D Coordinates: X={X}, Y={Y}, Z={Z}')
                        cv2.putText(frame, f'Cable X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

                    segmented_points = segment_cable(filtered_cable_contour, depth_frame, depth_intrin, connector_centers_3d[0] if connector_centers_3d else (0,0,0))

                    for point in segmented_points:
                        X, Y, Z = point
                        # rospy.loginfo(f'Segmented Cable 3D Point: X={X}, Y={Y}, Z={Z}')
                        cv2.circle(frame, (int(u), int(v)), 3, (255, 0, 0), -1)

            # Visualizzazione delle maschere
            cv2.imshow('Frame', frame)
            cv2.imshow('Connector Mask', mask_connector)
            cv2.imshow('Cable Mask', mask_cable)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown('User requested shutdown.')
                break

    def on_shutdown(self):
        # Save current parameters before shutdown
        values = get_trackbar_values()
        params = {
            'LH_Connector': values['l_h_connector'],
            'LS_Connector': values['l_s_connector'],
            'LV_Connector': values['l_v_connector'],
            'UH_Connector': values['u_h_connector'],
            'US_Connector': values['u_s_connector'],
            'UV_Connector': values['u_v_connector'],
            'LH_Cable': values['l_h_cable'],
            'LS_Cable': values['l_s_cable'],
            'LV_Cable': values['l_v_cable'],
            'UH_Cable': values['u_h_cable'],
            'US_Cable': values['u_s_cable'],
            'UV_Cable': values['u_v_cable'],
            'LH_Top': values['l_h_top'],
            'LS_Top': values['l_s_top'],
            'LV_Top': values['l_v_top'],
            'UH_Top': values['u_h_top'],
            'US_Top': values['u_s_top'],
            'UV_Top': values['u_v_top'],
            'LH_Bottom': values['l_h_bottom'],
            'LS_Bottom': values['l_s_bottom'],
            'LV_Bottom': values['l_v_bottom'],
            'UH_Bottom': values['u_h_bottom'],
            'US_Bottom': values['u_s_bottom'],
            'UV_Bottom': values['u_v_bottom'],
            'LH_Left': values['l_h_left'],
            'LS_Left': values['l_s_left'],
            'LV_Left': values['l_v_left'],
            'UH_Left': values['u_h_left'],
            'US_Left': values['u_s_left'],
            'UV_Left': values['u_v_left'],
            'LH_Right': values['l_h_right'],
            'LS_Right': values['l_s_right'],
            'LV_Right': values['l_v_right'],
            'UH_Right': values['u_h_right'],
            'US_Right': values['u_s_right'],
            'UV_Right': values['u_v_right'],
        }
        save_params(params)

        # Stop the pipeline and close windows
        self.pipeline.stop()
        cv2.destroyAllWindows()
        rospy.loginfo("Shutting down gracefully...")

if __name__ == '__main__':
    try:
        processor = ImageDepthProcessor()
        rospy.on_shutdown(processor.on_shutdown)
        processor.process_frames()
    except rospy.ROSInterruptException:
        pass
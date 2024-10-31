import cv2
import numpy as np
import rospy
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge
import pyrealsense2 as rs
from realsense import RealSenseCamera 
from config import load_params, save_params
from trackbars import setup_trackbars, load_saved_params, get_trackbar_values
from vision import detect_colored_sides, visualize_side_masks, segment_cable, publish_pose, find_connector_axis_3d, draw_3d_axis_on_frame, calculate_rotation_angle  # Aggiungi le nuove funzioni

class ImageDepthProcessor:
    def __init__(self):
        """Initialize the ImageDepthProcessor node."""
        rospy.init_node('image_depth_processor', anonymous=True)

        self.bridge = CvBridge()
        self.pose_pub = rospy.Publisher('cable_pose', Pose, queue_size=10)
        self.axis_pub = rospy.Publisher('connector_axis', Pose, queue_size=10)
        self.camera = RealSenseCamera()

        setup_trackbars()
        saved_params = load_params()
        load_saved_params(saved_params)

    def process_frames(self):
        """Process frames from the RealSense camera."""
        while not rospy.is_shutdown():
            color_frame, depth_frame = self.camera.get_frames()
            if not color_frame or not depth_frame:
                continue

            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            frame = np.asanyarray(color_frame.get_data())
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            values = get_trackbar_values()

            # Rilevamento dei lati colorati
            points_3d = detect_colored_sides(hsv, values, frame, depth_frame)
            combined_side_mask = visualize_side_masks(hsv, values)
            cv2.imshow('Combined Side Mask', combined_side_mask)

            # Maschera per il connettore
            lower_connector = np.array([values['l_h_connector'], values['l_s_connector'], values['l_v_connector']])
            upper_connector = np.array([values['u_h_connector'], values['u_s_connector'], values['u_v_connector']])
            mask_connector = cv2.inRange(hsv, lower_connector, upper_connector)

            contours_connector, _ = cv2.findContours(mask_connector, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            connector_centers_3d = []

            # Elaborazione dei contorni del connettore
            for contour in contours_connector:
                if cv2.contourArea(contour) > 100:
                    x, y, w, h = cv2.boundingRect(contour)
                    center = (x + w // 2, y + h // 2)

                    cv2.drawContours(frame, [contour], 0, (0, 255, 0), 2)
                    cv2.circle(frame, center, 5, (0, 255, 0), -1)

                    u, v = center
                    depth = depth_frame.get_distance(u, v)
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [u, v], depth)

                    if not math.isnan(X) and not math.isnan(Y) and not math.isnan(Z):
                        connector_centers_3d.append((X, Y, Z))
                        cv2.putText(frame, f'Connector X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}', (u + 10, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

            # Trova e disegna l'asse del connettore
            if len(connector_centers_3d) >= 2:
                point1, point2 = find_connector_axis_3d(connector_centers_3d)
                draw_3d_axis_on_frame(frame, point1, point2, depth_intrin)
                
                # Calcolo opzionale dell'angolo di rotazione
                angle = calculate_rotation_angle(point1, point2)
                # cv2.putText(frame, f'Rotation Angle: {angle:.2f} degrees', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Pubblica le coordinate dell'asse
                # rospy.loginfo(f"Publishing axis with points: {point1} and {point2}")
                self.publish_axis(point1, point2, angle)

            if connector_centers_3d:
                publish_pose(self.pose_pub, *connector_centers_3d[0])

            lower_cable = np.array([values['l_h_cable'], values['l_s_cable'], values['l_v_cable']])
            upper_cable = np.array([values['u_h_cable'], values['u_s_cable'], values['u_v_cable']])
            mask_cable = cv2.inRange(hsv, lower_cable, upper_cable)

            contours_cable, _ = cv2.findContours(mask_cable, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_cable:
                cable_contour = max(contours_cable, key=cv2.contourArea)

                x, y, w, h = cv2.boundingRect(cable_contour)
                roi = (x - 6 * w, y - 6 * h, x + 12 * w, y + 12 * h)

                filtered_cable_contour = [pt for pt in cable_contour if roi[0] <= pt[0][0] <= roi[2] and roi[1] <= pt[0][1] <= roi[3]]
                filtered_cable_contour = np.array(filtered_cable_contour).reshape((-1, 1, 2))

                if len(filtered_cable_contour) > 0:
                    cv2.drawContours(frame, [filtered_cable_contour], 0, (255, 0, 0), 2)

                    x, y, w, h = cv2.boundingRect(filtered_cable_contour)
                    cable_center = (x + w // 2, y + h // 2)
                    cv2.circle(frame, cable_center, 5, (0, 0, 255), -1)

                    u, v = cable_center
                    depth = depth_frame.get_distance(u, v)
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [u, v], depth)

                    if not math.isnan(X) and not math.isnan(Y) and not math.isnan(Z):
                        cv2.putText(frame, f'Cable X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

                    segmented_points = segment_cable(filtered_cable_contour, depth_frame, depth_intrin, connector_centers_3d[0] if connector_centers_3d else (0,0,0))

                    for point in segmented_points:
                        X, Y, Z = point
                        cv2.circle(frame, (int(u), int(v)), 3, (255, 0, 0), -1)

            cv2.imshow('Frame', frame)
            cv2.imshow('Connector Mask', mask_connector)
            cv2.imshow('Cable Mask', mask_cable)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown('User requested shutdown.')
                break

    def publish_axis(self, point1, point2, angle):
        """Publish the 3D coordinates of the axis and its angle."""
        if any(math.isnan(coord) for coord in point1) or any(math.isnan(coord) for coord in point2):
            rospy.logwarn("Nan values detected in axis points. Skipping pubblications")
            return 
        
        #rospy.loginfo(f"Publishing Axis: Point1: {point1}, Point2: {point2}, Angle: {angle}")
        axis_pose = Pose()
        
        # Setting start point
        axis_pose.position = Point(
            x=(point1[0] + point2[0]) / 2,  # X coordinate of axis midpoint
            y=(point1[1] + point2[1]) / 2,  # Y coordinate of axis midpoint
            z=(point1[2] + point2[2]) / 2   # Z coordinate of axis midpoint
        )

        # Convert angle to quaternion for rotation representation
        quaternion = Quaternion()
        quaternion.z = math.sin(math.radians(angle / 2))
        quaternion.w = math.cos(math.radians(angle / 2))
        axis_pose.orientation = quaternion

        self.axis_pub.publish(axis_pose)

    def on_shutdown(self):
        """Save parameters and shutdown the processor."""
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

        self.camera.stop()
        cv2.destroyAllWindows()
        rospy.loginfo("Shutting down gracefully...")

if __name__ == '__main__':
    try:
        processor = ImageDepthProcessor()
        rospy.on_shutdown(processor.on_shutdown)
        processor.process_frames()
    except rospy.ROSInterruptException:
        pass
import cv2
import numpy as np
import rospy
import math
import pyrealsense2 as rs
import tf2_ros
import message_filters

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from cv_bridge import CvBridge
from aruco_orientation.aruco_realsense import RealSenseCamera 
from aruco_orientation.aruco_config import load_params, save_params
from aruco_orientation.aruco_trackbars import setup_trackbars, load_saved_params, get_trackbar_values
from aruco_orientation.aruco_vision import segment_cable, publish_pose, find_connector_axis_3d, draw_3d_axis_on_frame, calculate_rotation_angle
from aruco_orientation.aruco_detection_filtered import ArucoDetector, ExponentialSmoothing
from sensor_msgs.msg import Image, CameraInfo
from pytransform3d.rotations import quaternion_from_euler


class ImageDepthProcessor:
    def __init__(self):
        """Initialize the ImageDepthProcessor node."""
        rospy.init_node('image_depth_processor', anonymous=True)
        rospy.sleep(1)

        self.theta_smoother = ExponentialSmoothing(alpha=0.1)

        rospy.loginfo("Waiting for camera info ...")
        rgb_info_msg = rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
        self.K = np.array(rgb_info_msg.K).reshape([3, -1])
        rospy.loginfo("Got info")

        self.bridge = CvBridge()
        self.pose_pub = rospy.Publisher('cable_pose', PoseStamped, queue_size=10)
        self.axis_pub = rospy.Publisher('connector_axis', Pose, queue_size=10)
        # self.camera = RealSenseCamera()
        self.marker_pose_pub = rospy.Publisher('aruco_marker_pose', Pose, queue_size=10)
        self.largest_area_pub = rospy.Publisher('largest_area_center', Pose, queue_size=10) 

        self.mask_pub = rospy.Publisher('/camera/debug/mask', Image, queue_size=0)  
        self.debug_pub = rospy.Publisher('/camera/debug/color', Image, queue_size=0)  

        # TF listener per trasformazioni
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        setup_trackbars()
        saved_params = load_params()
        load_saved_params(saved_params)

        self.marker_size = 0.01  # Misura del marker in metri
        self.aruco_detector = ArucoDetector(self.marker_size)

        # Variabili per memorizzare i punti dell'asse
        self.last_point1 = None
        self.last_point2 = None

        image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        pose_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        ats = message_filters.ApproximateTimeSynchronizer([image_sub, pose_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.process_frames)

    def quaternion_from_euler_custom(self, euler_angles):
        """Conversione da angoli di Eulero a quaternioni."""
        return quaternion_from_euler(euler_angles, 0, 1, 2, extrinsic=False)


    def process_frames(self, rgb_msg, depth_msg):
        """Process frames from the RealSense camera."""
        grasping_point_base = None
        theta = None
        # print("procesing frame")
        tmp = self.bridge.imgmsg_to_cv2(rgb_msg)
        color_frame = cv2.cvtColor(tmp.copy(), cv2.COLOR_RGB2BGR)
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg)
        timestamp = rgb_msg.header.stamp

        hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)

        values = get_trackbar_values()

        # Ottenere la trasformazione EE->Base con il timestamp del frame
        try:
            latest_transform = self.tf_buffer.lookup_transform("fr3_link0", "camera", rospy.Time(0), rospy.Duration(1.0))
            latest_time = latest_transform.header.stamp

            # Se il timestamp del messaggio è troppo avanti, usa l'ultimo disponibile
            if timestamp > latest_time:
                # rospy.logwarn(f"Requested timestamp {timestamp} is in the future. Using latest available transform.")
                timestamp = latest_time

            transform = self.tf_buffer.lookup_transform("fr3_link0", "camera", timestamp, rospy.Duration(1.0))
            # rospy.loginfo(f"Transform from fr3_link0 to camera: {transform}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # rospy.logwarn(f"Could not get transform from base to EE: {str(e)}")
            return

        # Creare la matrice di trasformazione T_EE->Base
        trans = transform.transform.translation
        rot = transform.transform.rotation
        T_camera_base = self.create_transformation_matrix(trans, rot)

        # Rilevamento dell'ArUco marker
        # import pdb
        # pdb.set_trace()
        rvecs, tvecs = self.aruco_detector.detect_aruco_marker(color_frame)

        if rvecs is not None and tvecs is not None:
            for i in range(len(rvecs)):
                smoothed_x = self.aruco_detector.smoother_x.smoothed_value
                smoothed_y = self.aruco_detector.smoother_y.smoothed_value
                Z = tvecs[i][0][2]

                # Calcolare l'angolo di rotazione attorno assi X e Z 
                angle_x_deg, angle_z_deg = self.aruco_detector.calculate_rotation_angles(rvecs[i])

                # Pubblicare i dati su ROS
                pose_msg = Pose()
                pose_msg.position.x = smoothed_x
                pose_msg.position.y = smoothed_y
                pose_msg.position.z = Z
                quaternion = self.calculate_quaternion_from_euler(0, angle_x_deg, angle_z_deg)
                pose_msg.orientation = quaternion
                self.marker_pose_pub.publish(pose_msg)

        # Rilevamento del connettore e del cavo
        lower_connector = np.array([values['l_h_connector'], values['l_s_connector'], values['l_v_connector']])
        upper_connector = np.array([values['u_h_connector'], values['u_s_connector'], values['u_v_connector']])
        mask_connector = cv2.inRange(hsv, lower_connector, upper_connector)

        contours_connector, _ = cv2.findContours(mask_connector, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        connector_centers_3d = []

        for contour in contours_connector:
            if cv2.contourArea(contour) > 20:
                x, y, w, h = cv2.boundingRect(contour)
                center = (x + w // 2, y + h // 2)

                cv2.drawContours(color_frame, [contour], 0, (0, 255, 0), 2)
                cv2.circle(color_frame, center, 5, (0, 255, 0), -1)

                u, v = center
                # import pdb
                # pdb.set_trace()
                Z = depth_frame[v,u]/1000

                # Se Z non è valido, usa il valore del frame precedente
                if Z <= 0 or math.isnan(Z):
                    # rospy.logwarn(f"Invalid depth (Z={Z}) at pixel ({u}, {v}), using previous valid point.")
                    if self.last_point1 is not None:
                        X, Y, Z = self.last_point1
                    else:
                        continue  # Se non ci sono punti precedenti validi, continua a ignorare questo punto
                
                else:
                    X = (u - self.K[0, 2]) * Z / self.K[0, 0]
                    Y = (v - self.K[1, 2]) * Z / self.K[1, 1]

                # rospy.loginfo(f"Center: ({u}, {v}), X: {X}, Y: {Y}, Z: {Z}")

                # Memorizza il punto valido
                self.last_point1 = np.array([X, Y, Z])

                if not math.isnan(X) and not math.isnan(Y) and not math.isnan(Z):
                    connector_centers_3d.append(np.array([X, Y, Z]))

        # Trova la macro area più grande e pubblica il suo centro
        if contours_connector:
            largest_contour = max(contours_connector, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            center = (x + w // 2, y + h // 2)

            u, v = center
            Z = depth_frame[v,u]/1000

            # Se Z non è valido, usa il valore del frame precedente
            if Z <= 0 or math.isnan(Z):
                # rospy.logwarn(f"Invalid depth (Z={Z}) at pixel ({u}, {v}), using previous valid point.")
                if self.last_point2 is not None:
                    X, Y, Z = self.last_point2
                else:
                    return  # Se non ci sono punti precedenti validi, salta questo frame
            else:
                X = (u - self.K[0, 2]) * Z / self.K[0, 0]
                Y = (v - self.K[1, 2]) * Z / self.K[1, 1]

            # rospy.loginfo(f"Macro area center: X={X}, Y={Y}, Z={Z}")

            # Memorizza il punto valido
            self.last_point2 = np.array([X, Y, Z])

            if not math.isnan(X) and not math.isnan(Y) and not math.isnan(Z):
                publish_pose(self.largest_area_pub, X, Y, Z)
                # cv2.putText(color_frame, f'Grasping Point X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}', (10, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Trasformare le coordinate del grasping point rispetto alla base del robot
                grasping_point_camera = np.array([X, Y, Z, 1])  # Coordinate omogenee rispetto alla camera
                grasping_point_base = np.dot(T_camera_base, grasping_point_camera)

                
                # Stampa le coordinate del grasping point rispetto alla base del robot
                X_base, Y_base, Z_base = grasping_point_base[:3]
                cv2.putText(color_frame, f'Grasping Point (Base) X={X_base:.2f}, Y={Y_base:.2f}, Z={Z_base:.2f}', (10, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Calcolo dell'orientamento e disegno dell'asse 3D
        if len(connector_centers_3d) >= 2:
            point1, point2 = find_connector_axis_3d(connector_centers_3d)

            self.last_point1, self.last_point2 = point1, point2
        elif self.last_point1 is not None and self.last_point2 is not None:
            point1, point2 = self.last_point1, self.last_point2

        if self.last_point1 is not None and self.last_point2 is not None:
            draw_3d_axis_on_frame(color_frame, self.last_point1, self.last_point2, self.K)

            # Calcolare l'orientamento sul piano XY dell'asse virtuale
            point1_base = np.dot(T_camera_base, np.array([self.last_point1[0], self.last_point1[1], self.last_point1[2], 1]))[:3]
            point2_base = np.dot(T_camera_base, np.array([self.last_point2[0], self.last_point2[1], self.last_point2[2], 1]))[:3]

            delta_x = point2_base[0] - point1_base[0]
            delta_y = point2_base[1] - point1_base[1]
            theta = math.atan2(delta_y, delta_x)

            # Smoothing
            smoothed_theta = self.theta_smoother.update(theta)



            # Convertire l'angolo in gradi e visualizzarlo sul frame
            theta_degrees = math.degrees(smoothed_theta)
            cv2.putText(color_frame, f'Orientation (Base) XY={theta_degrees:.2f} deg', (10, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.putText(color_frame, f'Orientation (Base) XY={smoothed_theta:.2f} rad', (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # Pubblicare l'orientamento e il punto di grasping
            if grasping_point_base is not None and smoothed_theta is not None:
                self.publish_grasping_point_and_orientation(grasping_point_base, theta_degrees, "fr3_link0", timestamp) # we are using deg convention 
                
                
            # Calcolo del punto centrale e dell'angolo di rotazione
            center_x = (self.last_point1[0] + self.last_point2[0]) / 2
            center_y = (self.last_point1[1] + self.last_point2[1]) / 2
            center_z = (self.last_point1[2] + self.last_point2[2]) / 2
            angle = calculate_rotation_angle(self.last_point1, self.last_point2)

            # Pubblicare l'asse e l'angolo su ROS
            self.publish_axis(self.last_point1, self.last_point2, angle)

        # print(color_frame)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB), encoding="rgb8"))
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask_connector))


        
    def publish_grasping_point_and_orientation(self, grasping_point_base, theta, frame_id, timestamp):
        """Pubblica il punto di grasping e l'orientamento sul piano XY."""
        grasping_msg = PoseStamped()

        grasping_msg.header.stamp = timestamp
        grasping_msg.header.frame_id = frame_id

        grasping_msg.pose.position = Point(*grasping_point_base[:3])
        
        euler_angles = np.array([0, 0, math.radians(theta)])  
        # euler_angles_deg = np.array([0, 0, theta])  
        # print(f"Euler angles (rad): {euler_angles}")
        # print(f"Euler angles (deg): {euler_angles_deg}")

        quat_array = self.quaternion_from_euler_custom(euler_angles) 
        # print(f"Quaternions: {quat_array}")

        # Creiamo un oggetto Quaternion per ROS
        grasping_msg.pose.orientation = Quaternion(x=quat_array[1], y=quat_array[2], z=quat_array[3], w=quat_array[0]) # Cambio ordine dei componenti 

        # Pubblica il messaggio
        self.pose_pub.publish(grasping_msg)


    def publish_axis(self, point1, point2, angle):
        """Publish the 3D coordinates of the axis and its angle."""
        if any(math.isnan(coord) for coord in point1) or any(math.isnan(coord) for coord in point2):
            rospy.logwarn("Nan values detected in axis points. Skipping publication")
            return 
        
        axis_pose = Pose()
        axis_pose.position = Point(
            x=(point1[0] + point2[0]) / 2,  # X coordinate of axis midpoint
            y=(point1[1] + point2[1]) / 2,  # Y coordinate of axis midpoint
            z=(point1[2] + point2[2]) / 2   # Z coordinate of axis midpoint
        )

        quaternion = Quaternion()
        quaternion.z = math.sin(math.radians(angle / 2))
        quaternion.w = math.cos(math.radians(angle / 2))
        axis_pose.orientation = quaternion

        self.axis_pub.publish(axis_pose)

    def calculate_quaternion_from_euler(self, roll, pitch, yaw):
        """Calcola un quaternione a partire dagli angoli di rollio, beccheggio e imbardata (X, Y, Z)."""
        qx = math.sin(math.radians(roll) / 2) * math.cos(math.radians(pitch) / 2) * math.cos(math.radians(yaw) / 2) - math.cos(math.radians(roll) / 2) * math.sin(math.radians(pitch) / 2) * math.sin(math.radians(yaw) / 2)
        qy = math.cos(math.radians(roll) / 2) * math.sin(math.radians(pitch) / 2) * math.cos(math.radians(yaw) / 2) + math.sin(math.radians(roll) / 2) * math.cos(math.radians(pitch) / 2) * math.sin(math.radians(yaw) / 2)
        qz = math.cos(math.radians(roll) / 2) * math.cos(math.radians(pitch) / 2) * math.sin(math.radians(yaw) / 2) - math.sin(math.radians(roll) / 2) * math.sin(math.radians(pitch) / 2) * math.cos(math.radians(yaw) / 2)
        qw = math.cos(math.radians(roll) / 2) * math.cos(math.radians(pitch) / 2) * math.cos(math.radians(yaw) / 2) + math.sin(math.radians(roll) / 2) * math.sin(math.radians(pitch) / 2) * math.sin(math.radians(yaw) / 2)
        
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def create_transformation_matrix(self, translation, rotation):
        """Crea una matrice di trasformazione 4x4 usando traslazione e quaternione."""
        T = np.eye(4)
        T[0:3, 3] = [translation.x, translation.y, translation.z]

        # Convertire il quaternione in una matrice di rotazione
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        R = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        T[0:3, 0:3] = R
        return T

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
        }
        save_params(params)

        self.camera.stop()
        cv2.destroyAllWindows()
        rospy.loginfo("Shutting down gracefully...")

if __name__ == '__main__':
    processor = ImageDepthProcessor()

    while(not rospy.is_shutdown()):
        rospy.spin()


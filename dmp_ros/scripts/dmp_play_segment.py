import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from geometry_msgs.msg import WrenchStamped, PoseStamped
from franka_msgs.msg import FrankaState
from movement_primitives.dmp import CartesianDMP
from dmp_ros.spiral_search import get_search_trajectory
import pytransform3d.rotations as pr

class DMP_ROS:

    def __init__(self, dmp_path, data_path):
        ############################################## ROS setup
        rospy.init_node("dmp_ros")
        
        self.sub_robot_state = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.state_cb, queue_size=10)
        self.sub_F_ext_base = rospy.Subscriber("/franka_state_controller/F_ext_base", WrenchStamped, self.wrench_cb, queue_size=10)

        self.pub_des_pose = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)

        self.ros_rate = 750
        self.rate = rospy.Rate(self.ros_rate)

        self.load_dmps(path=dmp_path)
        self.load_data(path=data_path)


    def publish_wrench(self, pub, base_frame, force, torque):
        msg = WrenchStamped()
        msg.header.frame_id = base_frame
        msg.header.stamp = rospy.Time.now()

        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]

        msg.wrench.torque.x = torque[0]
        msg.wrench.torque.y = torque[1]
        msg.wrench.torque.z = torque[2]

        pub.publish(msg)

    def publish_pose(self, pub, base_frame, position, orientation):
        msg = PoseStamped()
        msg.header.frame_id = base_frame
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]

        msg.pose.orientation.w = orientation[0]
        msg.pose.orientation.x = orientation[1]
        msg.pose.orientation.y = orientation[2]
        msg.pose.orientation.z = orientation[3]

        pub.publish(msg)

    def state_cb(self, state_msg):        
        self.O_T_EE = np.array(state_msg.O_T_EE).reshape(4, 4).T
        O_R_EE = self.O_T_EE[:3, :3]
        position_EE = self.O_T_EE[:3, 3]
        q_w, q_x, q_y, q_z = pr.quaternion_from_matrix(O_R_EE, strict_check=False)
        self.pose = np.array([[position_EE[0]],[position_EE[1]],[position_EE[2]], [q_w],[q_x],[q_y],[q_z]])

    def wrench_cb(self, msg: WrenchStamped):
        self.wrench = np.array([[msg.wrench.force.x], [msg.wrench.force.y], [msg.wrench.force.z],
                            [msg.wrench.torque.x], [msg.wrench.torque.y], [msg.wrench.torque.z]])
        
    def load_data(self, path):
        d = np.load(path)
        data = d['data']
        self.demo_time = d['time']

        self.demo_data = np.concatenate([
            data[:, :3],                    # x, y, z
            data[:, 6:7],                   # qw
            data[:, 3:6],                   # qx, qy, qz
            data[:, 7:]], axis=1)           # fx, fy, fz, tx, ty, tz
        
        segment_indices = 50*np.array([50, 113, 167, 230, 280, 363])
        self.segments = self.segment_data_and_time(self.demo_data, self.demo_time, segment_indices)

    def load_dmps(self, path):
        d = np.load(path)
        self.segmemnt_weights = d['dmp_weights']
        self.segment_durations = d['segment_durations']

    def segment_data_and_time(self, data_array, time_array, segment_indices):
        segments = []  # This will store tuples of (data_segment, time_segment)
        
        # Add a segment for the start if not starting from 0
        if segment_indices[0] != 0:
            segment_indices = np.hstack((0, segment_indices))
        
        # Add a segment for the end if not ending at the last index
        if segment_indices[-1] != len(data_array):
            segment_indices = np.hstack((segment_indices, len(data_array)))
        
        for i in range(1, len(segment_indices)-1):
            start_idx = segment_indices[i-1]
            end_idx = segment_indices[i]
            
            data_segment = data_array[start_idx:end_idx]
            time_segment = time_array[start_idx:end_idx]

            # reset time to zero
            time_segment = time_segment - time_segment[0]
            
            segments.append((data_segment, time_segment))
        
        return segments


if __name__ == '__main__':
    dmp_path = '/home/jheidersberger/catkin_ws/src/dmp_ros/data/dmps/' + 'teleop_recording_2024-02-08-14-02-49_Hole1_User1_Start3_dmp.npz'
    demo_path = '/home/jheidersberger/catkin_ws/src/dmp_ros/data/' + 'teleop_recording_2024-02-08-14-02-49_Hole1_User1_Start3.npz'

    d = DMP_ROS(dmp_path=dmp_path, data_path=demo_path)

    n_segments = d.segment_durations.shape[0]

    search_segment = 3 # manually selected

    version = 0 # 0=all dmps, 1=replace search with spiral, 2= append spiral to search

    slowdown_factor = 10

    for current_segment in range(n_segments):
        print(f"executing segment: {current_segment}")
        
        if current_segment == search_segment and version == 1:
            # TODO spiral pattern
            # start_pose = d.segments[current_segment+1][0, :7]
            start_pose = np.squeeze(d.pose)
            # Rearrange to [x, y, z, qx, qy, qz, qw]
            start_pose = [start_pose[0], start_pose[1], start_pose[2], start_pose[4], start_pose[5], start_pose[6], start_pose[3]]
            spiral_segment = get_search_trajectory(start_pose=start_pose, end_pose=None)
            pose_traj = np.hstack((spiral_segment[:, :3], spiral_segment[:, 6:7], spiral_segment[:, 3:6]))
        else:
            dmp = CartesianDMP(execution_time=d.segment_durations[current_segment], dt=1/d.ros_rate)
            dmp.set_weights(d.segmemnt_weights[current_segment])
            # Set initial and goal conditions
            # dmp.configure(start_y=d.segments[current_segment][0][0, :7], goal_y=d.segments[current_segment][0][-1, :7])
            dmp.configure(start_y=np.squeeze(d.pose), goal_y=d.segments[current_segment][0][-1, :7])
            

            # Run open loop
            time_traj, pose_traj = dmp.open_loop(d.segment_durations[current_segment]*slowdown_factor)

        for pose in pose_traj:
            d.publish_pose(d.pub_des_pose, 'fr3_link0', pose[:3], pose[3:])
            d.rate.sleep()
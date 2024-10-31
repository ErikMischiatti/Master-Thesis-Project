#!/usr/bin/env python3
import rospy
import yaml
import rospkg
from geometry_msgs.msg import PoseStamped
import numpy as np
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped
from franka_msgs.msg import FrankaState
from movement_primitives.dmp import CartesianDMP
from dmp_ros.spiral_search import get_search_trajectory
import pytransform3d.rotations as pr
import matplotlib.pyplot as plt

class DMP_ROS:

    def __init__(self):
        ############################################## ROS setup
        rospy.init_node("dmp_ros")

        self.config_path = rospy.get_param('~config_path', 'default/path/to/config.yaml')
        self.plot = rospy.get_param('~plot', 'false')
        
        self.sub_robot_state = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.state_cb, queue_size=10)
        self.sub_F_ext_base = rospy.Subscriber("/franka_state_controller/F_ext_base", WrenchStamped, self.wrench_cb, queue_size=10)

        # self.pub_des_pose = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
        self.pub_des_pose = rospy.Publisher('/cartesian_impedance_controller_damping_ratio/equilibrium_pose', PoseStamped, queue_size=10)

        self.pub_des_twist = rospy.Publisher('/cartesian_impedance_controller_damping_ratio/desired_velocity', TwistStamped, queue_size=10)

        self.ros_rate = 750
        self.rate = rospy.Rate(self.ros_rate)

        rospy.sleep(1.0)

        package_name='dmp_ros'
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)
        filename = config['file_name'] + ".npz"
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(package_name)
        file_path = f"{package_path}/data/dmps/{filename}"
        self.load_dmps(path=file_path)

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

    def publish_twist(self, pub, base_frame, linear, angular):
        msg = TwistStamped()
        msg.header.frame_id = base_frame
        msg.header.stamp = rospy.Time.now()

        msg.twist.linear.x = linear[0]
        msg.twist.linear.y = linear[1]
        msg.twist.linear.z = linear[2]

        msg.twist.angular.x = angular[0]
        msg.twist.angular.y = angular[1]
        msg.twist.angular.z = angular[2]

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

    def load_dmps(self, path):
        d = np.load(path)
        self.segmemnt_weights = d['dmp_weights']
        self.segment_durations = d['segment_durations']
        self.segment_start_poses = d['start_poses']
        self.segment_goal_poses = d['goal_poses']

        print(self.segment_start_poses.shape)

def main():
    d = DMP_ROS()
    play_dmp_segmentes(d)

def play_dmp_segmentes(d):
    n_segments = d.segment_durations.shape[0]

    search_segment = 1 # manually selected

    version = 2 # 0=all dmps, 1=replace search with spiral, 2= append spiral to search

    slowdown_factor = 1.5
    slowdown_distance_gain = 1.0

    segment_pause = 1.0

    dt = 1 / d.ros_rate

    vel_gain = 1.8

    pos_offset = np.array([-0.2, -0.50, 0.0])

    for current_segment in range(n_segments):
        print(f"executing segment: {current_segment}")
        
        if current_segment == search_segment and version == 1:
            # start_pose = d.segments[current_segment+1][0, :7]
            start_pose = d.segment_start_poses[current_segment,:]
            # Rearrange to [x, y, z, qx, qy, qz, qw]
            start_pose = [start_pose[0], start_pose[1], start_pose[2], start_pose[4], start_pose[5], start_pose[6], start_pose[3]]
            spiral_segment = get_search_trajectory(start_pose=start_pose, end_pose=None)
            pose_traj = np.hstack((spiral_segment[:, :3], spiral_segment[:, 6:7], spiral_segment[:, 3:6]))

            # Play motion
            for i, pose in enumerate(pose_traj):
                if i > 0:
                    lin_velocity = (pose_traj[i, :3] - pose_traj[i - 1, :3]) /dt *vel_gain
                    ang_velocity = pr.compact_axis_angle(pr.quaternion_diff(pose_traj[i, 3:], pose_traj[i - 1, 3:])) /dt
                else:
                    lin_velocity = np.zeros((3,1))
                    ang_velocity = np.zeros((3,1))

                d.publish_pose(d.pub_des_pose, 'fr3_link0', pose[:3], pose[3:])
                d.publish_twist(d.pub_des_twist, 'fr3_link0', lin_velocity, ang_velocity)
                d.rate.sleep()

        else:
            start_pose_demo = d.segment_start_poses[current_segment,:]
            goal_pose_demo = d.segment_goal_poses[current_segment,:]
            dmp_seg_goal = d.segment_goal_poses[current_segment,:]
            dmp_seg_goal[:3] = dmp_seg_goal[:3] + pos_offset

            effective_slowdown_factor = slowdown_factor + slowdown_distance_gain*(np.linalg.norm(dmp_seg_goal[:3]-np.squeeze(d.pose)[:3]) / np.linalg.norm(goal_pose_demo[:3]-start_pose_demo[:3]))
            print(f"effective slowdown factor: {np.round(effective_slowdown_factor, 3)}")

            dmp_duration = d.segment_durations[current_segment]*effective_slowdown_factor
            dmp = CartesianDMP(execution_time=dmp_duration, dt=1/d.ros_rate)
            dmp.set_weights(d.segmemnt_weights[current_segment])
            dmp.configure(start_y=np.squeeze(d.pose), goal_y=dmp_seg_goal)

            # Run open loop
            time_traj, pose_traj = dmp.open_loop()

            if d.plot:
                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')
                ax.plot(pose_traj[:, 0], pose_traj[:, 1], pose_traj[:, 2])
                ax.set_box_aspect([1,1,1])
                ax.set_aspect('equal')
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                plt.title('DMP motion')
                plt.show()

            # Play motion
            last_y = np.squeeze(d.pose)
            last_yd = np.zeros((6,))
            t = 0
            while t < dmp_duration:
                pose, twist = dmp.step(last_y=last_y, last_yd=last_yd)
                last_y = pose.copy()
                last_yd = twist.copy()

                d.publish_pose(d.pub_des_pose, 'fr3_link0', pose[:3], pose[3:])
                d.publish_twist(d.pub_des_twist, 'fr3_link0', twist[:3], twist[3:])
                d.rate.sleep()

                t = t + 1/d.ros_rate

            print(f"Control position error norm: {np.round(np.linalg.norm(pose[:3] - np.squeeze(d.pose)[:3]),3)}")

            # Append search segment
            if current_segment == search_segment-1 and version == 2:
                print(f"adding spiral search")
                start_pose = np.squeeze(d.pose)
                start_pose[:3] = start_pose[:3] + np.array([0, 0, -0.01])
                # Rearrange to [x, y, z, qx, qy, qz, qw]
                start_pose = [start_pose[0], start_pose[1], start_pose[2], start_pose[4], start_pose[5], start_pose[6], start_pose[3]]
                spiral_segment = get_search_trajectory(start_pose=start_pose, end_pose=None, spiral_separataion=0.02)
                # Rearrange to [x, y, z, qw, qx, qy, qz]
                pose_traj = np.hstack((spiral_segment[:, :3], spiral_segment[:, 6:7], spiral_segment[:, 3:6]))

                # Play motion
                for i, pose in enumerate(pose_traj):
                    if i > 0:
                        lin_velocity = (pose_traj[i, :3] - pose_traj[i - 1, :3]) /dt *vel_gain
                        ang_velocity = pr.compact_axis_angle(pr.quaternion_diff(pose_traj[i, 3:], pose_traj[i - 1, 3:])) /dt
                    else:
                        lin_velocity = np.zeros((3,1))
                        ang_velocity = np.zeros((3,1))

                    d.publish_pose(d.pub_des_pose, 'fr3_link0', pose[:3], pose[3:])
                    d.publish_twist(d.pub_des_twist, 'fr3_link0', lin_velocity, ang_velocity)
                    d.rate.sleep()


        rospy.sleep(segment_pause)

        print(f"\n \n DMP position error: {np.round(d.segment_goal_poses[current_segment,:3] - pose[:3],3)}, norm: {np.round(np.linalg.norm(d.segment_goal_poses[current_segment,:3] - pose[:3]),3)}")
        print(f"Control position error: {np.round(pose[:3] - np.squeeze(d.pose)[:3],3)}, norm: {np.round(np.linalg.norm(pose[:3] - np.squeeze(d.pose)[:3]),3)}")
        print(f"Total position error: {np.round(d.segment_goal_poses[current_segment,:3] - np.squeeze(d.pose)[:3],3)}, norm: {np.round(np.linalg.norm(d.segment_goal_poses[current_segment,:3] - np.squeeze(d.pose)[:3]),3)}\n\n")

if __name__ == '__main__':
    main()
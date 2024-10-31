#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import actionlib
import cartesian_control_msgs.msg
import matplotlib.pyplot as plt

from pytransform3d.rotations import matrix_from_euler
from dhb_ros.reconstructTrajectory import reconstructTrajectory
from scipy.spatial.transform import Rotation as R
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped


class DHB_ROS:
    def __init__(self):
        ############################################## ROS setup
        rospy.init_node("dhb_ros")
        
        self.motion = rospy.get_param('~motion')

        self.sub_robot_state = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.state_cb, queue_size=10)
        self.sub_F_ext_base = rospy.Subscriber("/franka_state_controller/F_ext_base", WrenchStamped, self.wrench_cb, queue_size=10)
        self.sub_gripper = rospy.Subscriber("/franka_gripper/joint_states", JointState, self.gripper_cb, queue_size=1)

        self.pub_des_pose = rospy.Publisher("/trajectory_cartesian_impedance_controller/equilibrium_pose", PoseStamped, queue_size=1)
        self.pub_alpha = rospy.Publisher("/alpha", Float64, queue_size=1)

        self.ros_rate = 750
        self.rate = rospy.Rate(self.ros_rate)

        self.client = actionlib.SimpleActionClient('/trajectory_cartesian_impedance_controller/follow_cartesian_trajectory', cartesian_control_msgs.msg.FollowCartesianTrajectoryAction)

        rospy.loginfo("Waiting for action server to start...")
        self.client.wait_for_server()
        rospy.loginfo("Action server started, sending goal.")

        rospy.sleep(1.0)

        self.pose = None 

    def publish_double(self, pub, value):
        msg = Float64()
        msg.data = value
        pub.publish(msg)

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

        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3]

        pub.publish(msg)

    def state_cb(self, state_msg):        
        self.O_T_EE = np.array(state_msg.O_T_EE).reshape(4, 4).T
        O_R_EE = self.O_T_EE[:3, :3]
        position_EE = self.O_T_EE[:3, 3]
        rpy = R.from_matrix(O_R_EE).as_euler('xyz')  # Convert matrix to RPY
        self.pose = np.hstack((position_EE, rpy))

    def wrench_cb(self, msg: WrenchStamped):
        self.wrench = np.array([[msg.wrench.force.x], [msg.wrench.force.y], [msg.wrench.force.z],
                                [msg.wrench.torque.x], [msg.wrench.torque.y], [msg.wrench.torque.z]])

    def gripper_cb(self, msg: JointState):
        positions = msg.position
        dist = sum(positions)
        if dist >= 0.07:
            self.gripper_state = 0
        else:
            self.gripper_state = 1

    def load_dhb(self, path):
        data = np.load(path)
        self.positions_learn = data['positions']
        self.orientations_learn = data['orientations']
        self.duration = data['duration']

    def create_traj_goal(self, times, pos):
        goal = cartesian_control_msgs.msg.FollowCartesianTrajectoryGoal()
        goal.trajectory.header.frame_id = "fr3_link0"

        goal.goal_tolerance.position_error.x = 0.01
        goal.goal_tolerance.position_error.y = 0.01
        goal.goal_tolerance.position_error.z = 0.01

        goal.goal_tolerance.orientation_error.x = 0.01
        goal.goal_tolerance.orientation_error.y = 0.01
        goal.goal_tolerance.orientation_error.z = 0.01

        for i, (time, p) in enumerate(zip(times[1:], pos[1:])):
            if len(p) < 6:
                rospy.logerr(f"Pose at index {i} does not have enough elements: {len(p)}")
                continue

            traj_point = cartesian_control_msgs.msg.CartesianTrajectoryPoint()
            traj_point.time_from_start = rospy.Duration(time)
            traj_point.pose.position.x = p[0]
            traj_point.pose.position.y = p[1]
            traj_point.pose.position.z = p[2]

            # Convert RPY to Quaternion
            # quaternion = R.from_euler('xyz', p[3:6]).as_quat()
            traj_point.pose.orientation.x = p[4]
            traj_point.pose.orientation.y = p[5]
            traj_point.pose.orientation.z = p[6]
            traj_point.pose.orientation.w = p[3]

            # Set to nan so the controller can compute the value on its own
            traj_point.twist.linear.x = float("NaN")
            traj_point.acceleration.angular.x = float("NaN")

            goal.trajectory.points.append(traj_point)

        return goal
    

if __name__ == '__main__':
    dhb_ros = DHB_ROS()
    dhb_ros.__init__()

    dhb_ros.publish_double(dhb_ros.pub_alpha, 0)

    slowdown_factor = 2.5

    # Load the trajectory generated from the previous script
    dhb_trajectory_file = '/home/asl_team/catkin_ws/src/dhb_ros-General/data/dhb/generated_traj_dhb_vel_starting_point_2_ee_velocities.npz' # CHANGE HERE 
    dhb_ros.load_dhb(dhb_trajectory_file)

        # <param name="traj_path" value='/home/asl_team/catkin_ws/src/dhb_ros-General/data/dhb/generated_traj_dhb_vel_starting_point_2_ee_velocities.npz' />
        # <param name="vel_path" value='/home/asl_team/catkin_ws/src/dhb_ros-General/data/velocities/starting_point_2_ee_velocities.npz' />
    
    dt = dhb_ros.duration * slowdown_factor / len(dhb_ros.positions_learn)
    time_traj = np.linspace(0, dhb_ros.duration * slowdown_factor, len(dhb_ros.positions_learn))

    positions_play = dhb_ros.positions_learn
    orientations_play = dhb_ros.orientations_learn
    print(positions_play.shape)
    print(orientations_play.shape)

    min_length = min(len(time_traj), len(positions_play))
    time_traj = time_traj[:min_length]
    positions_play = positions_play[:min_length]
    orientations_play = orientations_play[:min_length]

    for i, (pos, ori) in enumerate(zip(positions_play[:10], orientations_play[:10])):
        rospy.loginfo(f"positions_play[{i}]: {pos}, orientations_play[{i}]: {ori}")

    traj_points = np.hstack((positions_play, orientations_play))
    goal_traj = dhb_ros.create_traj_goal(time_traj, traj_points)
    dhb_ros.client.send_goal(goal_traj)
    rospy.loginfo("Goal sent to action server.")

    dhb_ros.client.wait_for_result()
    result = dhb_ros.client.get_result()

    rospy.loginfo("Trajectory execution finished with result: %s", result)

    if "pick" in dhb_ros.motion or "grab" in dhb_ros.motion or "grasp" in dhb_ros.motion:
        dhb_ros.gripper.close()
    elif "place" in dhb_ros.motion or "put" in dhb_ros.motion:
        dhb_ros.gripper.open()

    dhb_ros.rate.sleep()
    rospy.sleep(2)

    if dhb_ros.pose is not None:
        goal_pose = np.hstack((positions_play[-1], orientations_play[-1]))
        
        dmp_position_error = goal_pose[:3] - positions_play[-1]
        control_position_error = positions_play[-1] - dhb_ros.pose[:3]

        print(f"DMP position error: {dmp_position_error}, norm: {np.linalg.norm(dmp_position_error)}")
        print(f"Control position error: {control_position_error}, norm: {np.linalg.norm(control_position_error)}")
        print(f"Total position error: {dmp_position_error - control_position_error}, norm: {np.linalg.norm(dmp_position_error - control_position_error)}")
    else:
        rospy.logerr("Pose is not initialized. Ensure the state_cb is being called correctly.")
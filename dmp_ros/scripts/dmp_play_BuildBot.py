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
from dmp_ros.Gripper import Gripper
from sensor_msgs.msg import JointState

class DMP_ROS:

    def __init__(self):
        ############################################## ROS setup
        rospy.init_node("dmp_ros")

        self.config_path = rospy.get_param('~config_path', 'default/path/to/config.yaml')
        
        self.sub_robot_state = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.state_cb, queue_size=10)
        self.sub_F_ext_base = rospy.Subscriber("/franka_state_controller/F_ext_base", WrenchStamped, self.wrench_cb, queue_size=10)
        self.sub_gripper = rospy.Subscriber("/franka_gripper/joint_states", JointState, self.gripper_cb, queue_size=1)

        self.pub_des_pose = rospy.Publisher("/cartesian_impedance_controller_damping_ratio/equilibrium_pose", PoseStamped, queue_size=1)
        self.pub_des_twist = rospy.Publisher("/cartesian_impedance_controller_damping_ratio/desired_velocity", TwistStamped, queue_size=1)

        self.ros_rate = 750
        self.rate = rospy.Rate(self.ros_rate)

        self.gripper = Gripper()

        package_name='dmp_ros'
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)
        filename = config['file_name'] + ".npz"
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(package_name)
        file_path = f"{package_path}/data/dmps/{filename}"
        self.load_dmps(path=file_path)

        rospy.sleep(1.0)

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

        # msg.pose.orientation.w = orientation[3]
        # msg.pose.orientation.x = orientation[0]
        # msg.pose.orientation.y = orientation[1]
        # msg.pose.orientation.z = orientation[2]

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
        
    def gripper_cb(self, msg: JointState):
        postions = msg.position
        dist = sum(postions)
        if dist >= 0.07:
            self.gripper_state = 0
        else:
            self.gripper_state = 1

    def load_dmps(self, path):
        data = np.load(path)
        self.weights = data['weights']
        self.duration = data['duration']
        self.demo_goal = data['demo_goal']
        self.n_bfs = int(self.weights.shape[0]/6)

if __name__ == '__main__':
    dmp_ros = DMP_ROS()

    dmp_ros.gripper.open()
    if dmp_ros.gripper_state == 0:
        opened = True
        closed = False
    else:
        opened = False
        closed = True

    slowdown_factor = 1.0

    dmp = CartesianDMP(execution_time=dmp_ros.duration*slowdown_factor, dt=1/dmp_ros.ros_rate, n_weights_per_dim=dmp_ros.n_bfs, smooth_scaling=True)
    print(dmp_ros.weights)
    print(dmp_ros.weights.shape)
    dmp.set_weights(dmp_ros.weights)

    goal_pose_demo = dmp_ros.demo_goal
    goal_pose = goal_pose_demo.copy()
    goal_pose[:3] = goal_pose[:3] + np.array([0, 0, 0])
    goal_pose[3:] = np.array([0, 1, 0, 0])

    new_goal = np.array([0.525, 0.168, 0.035, 0, 1, 0, 0])
    goal_pose = new_goal

    dmp.configure(start_y=np.squeeze(dmp_ros.pose), goal_y=goal_pose)
    
    # Run open loop
    time_traj, pose_traj = dmp.open_loop()

    for i, pose in enumerate(pose_traj):
        dmp_ros.publish_pose(dmp_ros.pub_des_pose, 'fr3_link0', pose[:3], pose[3:])
        if i > 0.9*len(pose_traj):
            dmp_ros.gripper.close()
        dmp_ros.rate.sleep()

    rospy.sleep(2)
    dmp_ros.gripper.open()
    rospy.sleep(2)

    print(f"DMP position error: {goal_pose[:3] - pose[:3]}, norm: {np.linalg.norm(goal_pose[:3] - pose[:3])}")
    print(f"Control position error: {pose[:3] - np.squeeze(dmp_ros.pose)[:3]}, norm: {np.linalg.norm(pose[:3] - np.squeeze(dmp_ros.pose)[:3])}")
    print(f"Total position error: {goal_pose[:3] - np.squeeze(dmp_ros.pose)[:3]}, norm: {np.linalg.norm(goal_pose[:3] - np.squeeze(dmp_ros.pose)[:3])}")
import rospy
import actionlib
import tf
import cartesian_control_msgs.msg
from sensor_msgs.msg import JointState
import numpy as np
import pprint

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def create_traj_goal(times, pos):
        """
        Creates the trajectory message from a list of times, poses.
        
        Currently velocities and accelerations are not used dues to the problem with waypoint interpolation in
        the trajectory controller.
        """
        goal = cartesian_control_msgs.msg.FollowCartesianTrajectoryGoal()
        goal.trajectory.header.frame_id = "fr3_link0"

        goal.goal_tolerance.position_error.x = 0.01
        goal.goal_tolerance.position_error.y = 0.01
        goal.goal_tolerance.position_error.z = 0.01

        goal.goal_tolerance.orientation_error.x = 0.01
        goal.goal_tolerance.orientation_error.y = 0.01
        goal.goal_tolerance.orientation_error.z = 0.01

        for i,  (time, p) in enumerate(zip(times[1:], pos[1:])):

            # rospy.loginfo(f"Creating traj. point {i}: time={time}, position={p}")

            traj_point = cartesian_control_msgs.msg.CartesianTrajectoryPoint()
            traj_point.time_from_start = rospy.Duration(time)
            traj_point.pose.position.x = p[0] 
            traj_point.pose.position.y = p[1] 
            traj_point.pose.position.z = p[2] 
            traj_point.pose.orientation.x = p[3]
            traj_point.pose.orientation.y = p[4]
            traj_point.pose.orientation.z = p[5]
            traj_point.pose.orientation.w = p[6]

            # Set to nan so the controller can compute the value on it's own
            traj_point.twist.linear.x = float("NaN")
            traj_point.acceleration.angular.x = float("NaN")

            goal.trajectory.points.append(traj_point)

        return goal

def create_simple_traj_goal():
    goal = cartesian_control_msgs.msg.FollowCartesianTrajectoryGoal()
    goal.trajectory.header.frame_id = "fr3_link0"

    traj_point = cartesian_control_msgs.msg.CartesianTrajectoryPoint()
    traj_point.time_from_start = rospy.Duration(2.0)
    traj_point.pose.position.x = 0.5
    traj_point.pose.position.y = 0.0
    traj_point.pose.position.z = 0.5
    traj_point.pose.orientation.x = 0.0
    traj_point.pose.orientation.y = 0.0
    traj_point.pose.orientation.z = 0.0
    traj_point.pose.orientation.w = 1.0

    goal.trajectory.points.append(traj_point)

    traj_point2 = cartesian_control_msgs.msg.CartesianTrajectoryPoint()
    traj_point2.time_from_start = rospy.Duration(4.0)
    traj_point2.pose.position.x = 0.5
    traj_point2.pose.position.y = 0.1
    traj_point2.pose.position.z = 0.5
    traj_point2.pose.orientation.x = 0.0
    traj_point2.pose.orientation.y = 0.0
    traj_point2.pose.orientation.z = 0.0
    traj_point2.pose.orientation.w = 1.0

    goal.trajectory.points.append(traj_point2)

    return goal



if __name__ == "__main__":
    rospy.init_node("replay")
    client = actionlib.SimpleActionClient('/trajectory_cartesian_impedance_controller/follow_cartesian_trajectory', cartesian_control_msgs.msg.FollowCartesianTrajectoryAction)

    rospy.loginfo("Waiting for action server to start...")
    client.wait_for_server()
    rospy.loginfo("Action server started, sending goal.")

    # Parse the bag file to 
    data = np.load('/home/catkin_ws/src/nist_cables/processed_new/clip_closure_pos1_record_1.npz')
    ee_traj = data['ee_traj']
    times =  np.linspace(0, data['time'][-1], len(ee_traj)) # linearization of the time 
    # times = data['time']

    # traj_len = min(len(ee_traj), len(times))

    # ee_traj = ee_traj[1:traj_len]
    # times = times[1:traj_len]

    # ee_traj = ee_traj[::50]
    # times = times[::50]

    # Plotting the traj.
    plt.figure()
    plt.plot(times)
    plt.xlabel("Index")
    plt.ylabel("Time")
    plt.title("Time vs Index")


    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot(ee_traj[:, 0], ee_traj[:, 1], ee_traj[:, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("3D Trajectory Plot")
    plt.show()


    # import pdb
    # pdb.set_trace()

    rospy.loginfo(f"Loaded trajectory with {len(ee_traj)} points.")
    rospy.loginfo(f"First few trajectory points: {ee_traj[:5]}")
    rospy.loginfo(f"First few time points: {times[:5]}")


    # Create the goal of the traj.
    traj_goal = create_traj_goal(times, ee_traj)
    
    # TEST TRAJECTORY
    # traj_goal = create_simple_traj_goal() 

    # Debugging: print a portion of the goal 
    # rospy.loginfo("Generated trajectory goal: %s", pprint.pformat(traj_goal))


    # Sending the goal to the client 
    client.send_goal(traj_goal)
    rospy.loginfo("Goal sent to action server.")

    # Waiting the result 
    client.wait_for_result()
    result = client.get_result()

    rospy.loginfo("Trajectory execution finished with result: %s", result)


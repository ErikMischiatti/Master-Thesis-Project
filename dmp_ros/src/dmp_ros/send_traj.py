import cartesian_control_msgs.msg
import rospy
import actionlib

import numpy

def create_traj_goal(times, pos, vel, acc):
        goal = cartesian_control_msgs.msg.FollowCartesianTrajectoryGoal()
        goal.trajectory.header.frame_id = "fr3_link0"

        goal.goal_tolerance.position_error.x = 0.01
        goal.goal_tolerance.position_error.y = 0.01
        goal.goal_tolerance.position_error.z = 0.01

        goal.goal_tolerance.orientation_error.x = 0.01
        goal.goal_tolerance.orientation_error.y = 0.01
        goal.goal_tolerance.orientation_error.z = 0.01

        for time, p,v,a in zip(times[1:], pos[1:], vel[1:], acc[1:]):
            traj_point = cartesian_control_msgs.msg.CartesianTrajectoryPoint()
            traj_point.time_from_start = rospy.Duration(time)
            traj_point.pose.position.x = p[0] 
            traj_point.pose.position.y = p[1] 
            traj_point.pose.position.z = p[2] 
            traj_point.pose.orientation.x = p[4]
            traj_point.pose.orientation.y = p[5]
            traj_point.pose.orientation.z = p[6]
            traj_point.pose.orientation.w = p[3]

            traj_point.twist.linear.x = float("NaN")
            traj_point.acceleration.angular.x = float("NaN")

            # traj_point.twist.linear.x = v[0] 
            # traj_point.twist.linear.y = v[1] 
            # traj_point.twist.linear.z = v[2] 
            # traj_point.twist.angular.x = v[3]
            # traj_point.twist.angular.y = v[4]
            # traj_point.twist.angular.z = v[5]

            # traj_point.acceleration.linear.x = a[0] 
            # traj_point.acceleration.linear.y = a[1] 
            # traj_point.acceleration.linear.z = a[2] 
            # traj_point.acceleration.angular.x = a[3]
            # traj_point.acceleration.angular.y = a[4]
            # traj_point.acceleration.angular.z = a[5]

            goal.trajectory.points.append(traj_point)

        return goal


rospy.init_node("trajecotry_commander")

client = actionlib.SimpleActionClient('/trajectory_cartesian_impedance_controller/follow_cartesian_trajectory', cartesian_control_msgs.msg.FollowCartesianTrajectoryAction)
# client = actionlib.SimpleActionClient('/visualize_cartesian_trajectories/follow_cartesian_trajectory', cartesian_control_msgs.msg.FollowCartesianTrajectoryAction)
print("waiting for actions server")
client.wait_for_server()
print("Got server")

# load dmp and generate trajectory



# commend trajecotry
# traj_goal = create_traj_goal(times_dmp, traj_ee, np.zeros_like(traj_ee), np.zeros_like(traj_ee))

# client.send_goal(traj_goal)
# client.wait_for_result()

# res = client.get_result()
# print(res)


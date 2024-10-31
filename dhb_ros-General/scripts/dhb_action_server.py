#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
from dhb_ros.msg import DHBActionAction, DHBActionFeedback, DHBActionResult
import  dhb_ros
import cartesian_control_msgs.msg
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction
from dhb_ros.dhb import DHB
import tf
from scipy.interpolate import interp1d

from stencils import Stencil

import os
import matplotlib.pyplot as plt


def low_pass_filter(input_signal, cutoff_freq, sampling_freq):
    """
    Apply a first-order low-pass filter to the input signal.
    
    Parameters:
    - input_signal: A list or numpy array of input samples.
    - cutoff_freq: The cutoff frequency of the filter in Hz.
    - sampling_freq: The sampling frequency in Hz.
    
    Returns:
    - A list of filtered output samples.
    """
    # Calculate the alpha coefficient
    alpha = np.exp(-2 * np.pi * cutoff_freq / sampling_freq)
    print("aplha", alpha)
    # Initialize the previous output (y[n-1]) to zero
    prev_y = 0.0
    
    # List to store the filtered output
    filtered_signal = []
    
    # Apply the filter to each input sample
    for x in input_signal:
        # Compute the current output y[n] based on the difference equation
        y = (1 - alpha) * x + alpha * prev_y
        
        # Store the output in the list
        filtered_signal.append(y)
        
        # Update the previous output for the next iteration
        prev_y = y
    
    return filtered_signal

def postprocess_trajectory(time, positions, freq=200, stencil_size=3, cutoff_freq=50, filter_order=1):
    duration = time[-1]

    # Interpolation
    f = interp1d(time, positions.T, kind='cubic')

    # New finer sampling
    t_new = np.linspace(0, duration, int(duration*freq))
    y_new = f(t_new).T

    # Determine padding size, half on each side for symmetric padding
    pad_stencil = 2*stencil_size

    # Pad the data symmetrically
    y_padded = np.pad(y_new, (pad_stencil, ), mode='edge')[:, pad_stencil:-pad_stencil]
    t_new_padded = np.linspace(-pad_stencil * 0.001, duration + pad_stencil * 0.001, len(y_padded))

    # Filtering the padded data
    offset = y_padded[0]
    y_filtered = low_pass_filter(y_padded-offset, cutoff_freq, freq)
    y_filtered += offset

    # Instantiate stencil and compute derivatives
    stencil = Stencil(list(range(-stencil_size, stencil_size + 1)))
    dy = stencil.derive(t_new_padded, y_filtered, 1)  # Ensure the sizes match
    ddy = stencil.derive(t_new_padded, dy, 1)

    return t_new_padded[pad_stencil:-pad_stencil], y_padded[pad_stencil:-pad_stencil], dy[pad_stencil:-pad_stencil], ddy[pad_stencil:-pad_stencil]

def extend_trajectory(trajectory, times, f_i, f_o):
    """
    Extend the trajectory and times to a higher frequency by repeating samples.

    Parameters:
    - trajectory: List of trajectory points.
    - times: List of time points corresponding to the trajectory points.
    - f_i: Initial frequency of the trajectory (Hz).
    - f_o: Desired output frequency of the trajectory (Hz).

    Returns:
    - extended_trajectory: List of extended trajectory points.
    - extended_times: List of extended time points.
    """
    # Calculate the repetition factor
    repetition_factor = int(f_o / f_i)
    
    # Initialize extended trajectory and times
    extended_trajectory = []
    extended_times = []
    
    # Loop through the original trajectory and times
    for i in range(len(trajectory) - 1):
        point = trajectory[i]
        time_start = times[i]
        time_end = times[i + 1]
        
        # Calculate time increment for repeated samples
        time_increment = (time_end - time_start) / repetition_factor
        
        # Repeat the point and adjust times
        for j in range(repetition_factor):
            extended_trajectory.append(point)
            extended_times.append(time_start + j * time_increment)
    
    # Append the last point of the original trajectory and time
    extended_trajectory.append(trajectory[-1])
    extended_times.append(times[-1])
    
    return extended_times, extended_trajectory

class DHBActionServer:
    _feedback = DHBActionFeedback()
    _result = DHBActionResult()
    
    def __init__(self):
        rospy.init_node("dhb_action_server")

        rospy.sleep(2)
        self.action_name = rospy.get_param("~name")
        # self.dhb_file = rospy.get_param("~dhb_file")
        # self.trajectory_file = rospy.get_param("~traj_path")
        # self.vel_file = rospy.get_param("~vel_path")

        self.client = actionlib.SimpleActionClient('/trajectory_cartesian_impedance_controller/follow_cartesian_trajectory', cartesian_control_msgs.msg.FollowCartesianTrajectoryAction)
        # self.client = actionlib.SimpleActionClient('/cartesian_impedance_controller_damping_ratio/follow_cartesian_trajectory', cartesian_control_msgs.msg.FollowCartesianTrajectoryAction)


        self.listener = tf.TransformListener()

        # Action Server Setup
        self._as = actionlib.SimpleActionServer(self.action_name, DHBActionAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def create_traj_goal(self, times, poses, vels, accels):
        goal = cartesian_control_msgs.msg.FollowCartesianTrajectoryGoal()
        # import pdb
        # pdb.set_trace()
        # goal.header.frame_id = "panda_link0"
        goal.trajectory.header.frame_id = "panda_link0"
        # goal.goal_time_tolerance = rospy.Duration(1)

        goal.goal_tolerance.position_error.x = 0.07
        goal.goal_tolerance.position_error.y = 0.07
        goal.goal_tolerance.position_error.z = 0.07
        goal.goal_tolerance.orientation_error.x = 0.07
        goal.goal_tolerance.orientation_error.y = 0.07
        goal.goal_tolerance.orientation_error.z = 0.07

        goal.goal_tolerance.twist_error.linear.x = 0.07
        goal.goal_tolerance.twist_error.linear.y = 0.07
        goal.goal_tolerance.twist_error.linear.z = 0.07
        goal.goal_tolerance.twist_error.angular.x = 0.07
        goal.goal_tolerance.twist_error.angular.y = 0.07
        goal.goal_tolerance.twist_error.angular.z = 0.07

        goal.goal_tolerance.acceleration_error.linear.x = 0.07
        goal.goal_tolerance.acceleration_error.linear.y = 0.07
        goal.goal_tolerance.acceleration_error.linear.z = 0.07
        goal.goal_tolerance.acceleration_error.angular.x = 0.07
        goal.goal_tolerance.acceleration_error.angular.y = 0.07
        goal.goal_tolerance.acceleration_error.angular.z = 0.07

        for time, p, v, a in zip(times[1:], poses[1:], vels[1:], accels[1:]):
            traj_point = cartesian_control_msgs.msg.CartesianTrajectoryPoint()
            traj_point.time_from_start = rospy.Duration(time)
            traj_point.pose.position.x = p[0] #plt.plot(motion[:,0, 0], label="x position")
            traj_point.pose.position.y = p[1] #plt.plot(motion[:,0, 1], label="y position")
            traj_point.pose.position.z = p[2] #plt.plot(motion[:,0, 2], label="z position")
            traj_point.pose.orientation.w = p[3]
            traj_point.pose.orientation.x = p[4]
            traj_point.pose.orientation.y = p[5]
            traj_point.pose.orientation.z = p[6]
            

            traj_point.twist.linear.x = v[0] #plt.plot(motion[:,1, 0], label="x vel")
            traj_point.twist.linear.y = v[1] #plt.plot(motion[:,1, 1], label="y vel")
            traj_point.twist.linear.z = v[2] #plt.plot(motion[:,1, 2], label="z vel")
            traj_point.twist.angular.x = v[3]
            traj_point.twist.angular.y = v[4]
            traj_point.twist.angular.z = v[5]

            traj_point.acceleration.linear.x = a[0] #plt.plot(motion[:,2, 0], label="x acc")
            traj_point.acceleration.linear.y = a[1] #plt.plot(motion[:,2, 1], label="y acc")
            traj_point.acceleration.linear.z = a[2] #plt.plot(motion[:,2, 2], label="z acc")
            traj_point.acceleration.angular.x = a[3]
            traj_point.acceleration.angular.y = a[4]
            traj_point.acceleration.angular.z = a[5]

            goal.trajectory.points.append(traj_point)
        return goal
    


    def execute_cb(self, goal):
        rospy.loginfo(f"Received goal: slowdown_factor={goal.slowdown_factor}")
        rospy.loginfo(f"Received goal: goal={goal.goal}")


        # Carica i file di traiettoria e velocità
        self.dhb_ros = DHB(goal.dhb_file)  # Inizializza la classe DHB_ROS per accedere a tutte le funzioni

        goal_np = np.array([goal.goal.position.x, goal.goal.position.y, goal.goal.position.z,
                            goal.goal.orientation.w, goal.goal.orientation.x,
                            goal.goal.orientation.y, goal.goal.orientation.z])
        
        now = rospy.Time.now()

        try:
            self.listener.waitForTransform("/fr3_link0", "/fr3_EE", now, rospy.Duration(1))
            (trans, rot) = self.listener.lookupTransform("/fr3_link0", "/fr3_EE", now)
            start = np.array([*trans, rot[3], rot[0], rot[1], rot[2]])
            # start = np.array([*trans, *rot])
            # rospy.loginfo(f"Start pose: {start}")
        except:
            rospy.logerr("Unable to get ee pose")
            self._as.set_aborted(self._result)
            return

        rospy.loginfo(f"Start_AS: {start}")
        rospy.loginfo(f"Goal_AS: {goal_np}")

        trajectory_name = os.path.splitext(os.path.basename(goal.dhb_file))[0]

        time_traj, filtered_positions, filteres_scaled_orientations_quat = self.dhb_ros.generate_traj(start, goal_np, goal.slowdown_factor, verbose=True, trajectory_name=trajectory_name)

        traj_points_filtered = [np.hstack((pos, quat)) for pos, quat in zip(filtered_positions, filteres_scaled_orientations_quat)]
        traj_points_filtered = np.array(traj_points_filtered)


        N = 500
        # Determine the gap between points in the original trajectory
        gap = time_traj[-1] - time_traj[-2]

        # Generate the new points
        new_points = time_traj[-1] + np.arange(1, N+1) * gap

        # Append the new points to the original array
        extended_time_array = np.append(time_traj, new_points)

        # import pdb
        # pdb.set_trace()

        traj_points_filtered = np.vstack((traj_points_filtered, np.tile(traj_points_filtered[-1], (N, 1))))

        t_new, y_new, dy, ddy = postprocess_trajectory(extended_time_array, traj_points_filtered, freq=len(time_traj)/time_traj[-1], stencil_size=3)

        # plt.plot(t_new, y_new, label="pose")
        # plt.figure()
        # plt.plot(t_new, dy, label="vel")
        # plt.figure()
        # plt.plot(t_new, ddy, label="accel")
        # plt.show()

        # import pdb
        # pdb.set_trace()
        # print("AS distance", traj_points_filtered[0] - start)
        
        goal_traj_filtered = self.create_traj_goal(t_new, y_new, dy, ddy)

        # import pdb
        # pdb.set_trace()

        rospy.loginfo("Sending filtered trajectory goal to the client.")
        self.client.send_goal(goal_traj_filtered)
        self.client.wait_for_result()
        result = self.client.get_result()

        if result is not None:
            rospy.loginfo(f"Result received from client: {result}")
        else:
            rospy.logwarn("No result received from client!")

        if result.error_code == 0:
            rospy.loginfo('%s: Succeeded' % self.action_name)
            self._as.set_succeeded(self._result)
        else:
            rospy.logerr(f"{self.action_name}: Failed with error code: {result.error_code}")

        



def is_valid_quaternion(orientation):
    return not (orientation[3] == 0 and orientation[1] == 0 and orientation[2] == 0 and orientation[0] == 1)

if __name__ == '__main__':
    server = DHBActionServer()
    rospy.spin()
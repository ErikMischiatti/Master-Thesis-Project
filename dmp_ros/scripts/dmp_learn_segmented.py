#!/usr/bin/env python3
import numpy as np
import rospy
import yaml
import rospkg
from movement_primitives.dmp import CartesianDMP
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class CartesianDMPTrainer:
    def __init__(self, config_path, visualize=False):
        package_name='dmp_ros'
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        self.filename = self.config['file_name'] + ".npz"
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path(package_name)
        self.file_path = f"{self.package_path}/data/{self.filename}"        
        self.segmentation_indices = self.config.get('segmentation_indices', None)
        self.visualize = visualize

    def load_trajectory(self):
        # assumes quaternions are given in qx,qy,qz,qw order
        data = np.load(self.file_path) 
        adjusted_data = np.concatenate([
            data['data'][:, :3],                    # x, y, z
            data['data'][:, 6:7],                   # qw
            data['data'][:, 3:6]],                  # qx, qy, qz
            axis=1)
        return adjusted_data, data['time']

    def segment_data_and_time(self, data_array, time_array, segment_indices):
        segments = []
        print(f"segment_indices: {segment_indices}", flush=True)
        print(f"data_array.shape: {data_array.shape}", flush=True)
        rospy.loginfo(f"segment_indices: {segment_indices}")
        rospy.loginfo(f"data_array.shape: {data_array.shape}")
        if segment_indices[-1] > len(data_array):
            raise ValueError("Last segment index out of demo range")
        if segment_indices[0] != 0:
            segment_indices = np.hstack((0, segment_indices))
        if segment_indices[-1] != len(data_array):
            segment_indices = np.hstack((segment_indices, len(data_array)))        
        for i in range(1, len(segment_indices)):
            start_idx = segment_indices[i-1]
            end_idx = segment_indices[i]            
            data_segment = data_array[start_idx:end_idx]
            time_segment = time_array[start_idx:end_idx]            
            time_segment = time_segment - time_segment[0]            
            segments.append((data_segment, time_segment))        
        return segments

    def visualize_segment(self, data_segment, pose_dmp, segment_index):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(data_segment[:, 0], data_segment[:, 1], data_segment[:, 2], label=f'Demo Segment {segment_index+1}')
        
        ax.plot(pose_dmp[:, 0], pose_dmp[:, 1], pose_dmp[:, 2], label=f'DMP Segment {segment_index+1}', linestyle='--')
        
        ax.set_box_aspect([1,1,1]) 
        ax.set_aspect('equal')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.legend()
        plt.show()

    def visualize_all_segments(self, segments):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for segment_index, (data_segment, _) in enumerate(segments):
            ax.plot(data_segment[:, 0], data_segment[:, 1], data_segment[:, 2], label=f'Segment {segment_index+1}')
        
        ax.set_box_aspect([1,1,1])
        ax.set_aspect('equal')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.title('All Demo Segments')
        ax.legend()
        plt.show()
        
    def train_dmp_for_segments(self, segments):
        dmp_weights = []
        segment_durations = []
        start_poses = []
        goal_poses = []
        for segment_index, (data_segment, time_segment) in enumerate(segments):
            start_pose = data_segment[0]
            goal_pose = data_segment[-1]
            duration_segment = time_segment[-1]
            dt = np.mean(np.diff(time_segment))
            dmp_segment = CartesianDMP(execution_time=duration_segment, dt=dt, n_weights_per_dim=10)
            # dmp_segment = CartesianDMP(execution_time=1, dt=dt, n_weights_per_dim=10)
            dmp_segment.imitate(time_segment, data_segment)
            times_dmp, pose_dmp = dmp_segment.open_loop()
            weights = dmp_segment.get_weights()
            dmp_weights.append(weights)
            segment_durations.append(duration_segment)
            start_poses.append(start_pose)
            goal_poses.append(goal_pose)
            if self.visualize:
                self.visualize_segment(data_segment, pose_dmp, segment_index)
        return dmp_weights, segment_durations, start_poses, goal_poses
    
    def load_and_train(self):
        rospy.loginfo("Starting load_and_train")
        data, time = self.load_trajectory()
        if self.segmentation_indices is not None:
            segments = self.segment_data_and_time(data, time, np.array(self.segmentation_indices))
        else:
            segments = [(data, time - time[0])]  # Single segment, reset time to start at 0
        if self.visualize:
            self.visualize_all_segments(segments)
        dmp_weights, segment_durations, start_poses, goal_poses = self.train_dmp_for_segments(segments)
        weights_file_path = f"{self.package_path}/data/dmps/{self.filename.replace('.npz', '_dmp_weights.npz')}"
        # np.savez(weights_file_path, dmp_weights=dmp_weights, segment_durations=segment_durations, start_poses=start_poses, goal_poses=goal_poses)

def main():
    rospy.init_node('cartesian_dmp_trainer', anonymous=True, log_level=rospy.INFO)
    config_path = rospy.get_param('~config_path', 'default/path/to/config.yaml')
    visualize = rospy.get_param('~visualize', False)
    trainer = CartesianDMPTrainer(config_path, visualize=visualize)
    trainer.load_and_train()
    rospy.loginfo("DMP training completed")

if __name__ == '__main__':
    main()

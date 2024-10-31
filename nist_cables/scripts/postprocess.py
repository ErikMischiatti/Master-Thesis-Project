import bagpy
import argparse
import glob
import os
import pandas as pd
import cv2
import numpy as np
import rosbag
import ast
import matplotlib.pyplot as plt

from tf.transformations import quaternion_from_matrix, translation_from_matrix

parser = argparse.ArgumentParser()
parser.add_argument("raw_dir", type=str, help="path to the raw data directory")
parser.add_argument("output",  type=str, help="path to the output directory")

args = parser.parse_args()

def decompress_image(data):
    # Convert string representation of bytes to actual bytes
    if isinstance(data, str):
        data = ast.literal_eval(data)

    # Convert to a NumPy array using OpenCV
    image_np = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    return image_np

def extract_data(name, row):
    data = {}
    if "franka_states" in name:
        trans_mtx = np.array([[row["O_T_EE_0"], row["O_T_EE_4"], row["O_T_EE_8"], row["O_T_EE_12"]],
                              [row["O_T_EE_1"], row["O_T_EE_5"], row["O_T_EE_9"], row["O_T_EE_13"]],
                              [row["O_T_EE_2"], row["O_T_EE_6"], row["O_T_EE_10"], row["O_T_EE_14"]],
                              [row["O_T_EE_3"], row["O_T_EE_7"], row["O_T_EE_11"], row["O_T_EE_15"]]])
        
        rot = quaternion_from_matrix(trans_mtx) # (x,y,z,w)
        rot = [rot[3], rot[0], rot[1], rot[2]] # (w,x,y,z)
        trans = translation_from_matrix(trans_mtx)

        data["ee_pose"] = [*trans, *rot]
        data["joints"] = [row["q_0"], row["q_1"], row["q_2"], row["q_3"], row["q_4"], row["q_5"], row["q_6"]]

        return data
    
    if "gripper" in name:
        data["gripper"] = [row["position_0"], row["position_1"]]
        return data
    
    if "F_ext" in name:
        data["est_force"] = [row["wrench.force.x"], row["wrench.force.y"], row["wrench.force.z"]]
        data["est_torque"] = [row["wrench.torque.x"], row["wrench.torque.y"], row["wrench.torque.z"]]
        return data
    
    if "ft_raw" in name:
        data["measured_force"] = [row["wrench.force.x"], row["wrench.force.y"], row["wrench.force.z"]]
        data["measured_torque"] = [row["wrench.torque.x"], row["wrench.torque.y"], row["wrench.torque.z"]]
        return data
    
    if "ft_compensated" in name and "ft_compensated_base" not in name:
        data["compensated_force"] = [row["wrench.force.x"], row["wrench.force.y"], row["wrench.force.z"]]
        data["compensated_torque"] = [row["wrench.torque.x"], row["wrench.torque.y"], row["wrench.torque.z"]]
        return data
    
    if "ft_compensated_base" in name:
        data["compensated_base_force"] = [row["wrench.force.x"], row["wrench.force.y"], row["wrench.force.z"]]
        data["compensated_base_torque"] = [row["wrench.torque.x"], row["wrench.torque.y"], row["wrench.torque.z"]]
        return data

    if "ft_zeroed" in name:
        data["zeroed_force"] = [row["wrench.force.x"], row["wrench.force.y"], row["wrench.force.z"]]
        data["zeroed_torque"] = [row["wrench.torque.x"], row["wrench.torque.y"], row["wrench.torque.z"]]
        return data

    else:
        print(f"Unrecognized topic name: {name}")
        return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("raw_dir", type=str, help="path to the raw data directory")
    parser.add_argument("output", type=str, help="path to the output directory")
    args = parser.parse_args()

    bag_files = glob.glob(os.path.join(args.raw_dir, "**", "*.bag"), recursive=True)
    
    for file in bag_files:
        if "clip_closure_pos3" not in file:
            continue
        b = bagpy.bagreader(file)
        csv_files = [b.message_by_topic(topic) for topic in b.topics]
        data_frames = [pd.read_csv(file_name) for file_name in csv_files]
        
        file_name = os.path.splitext(os.path.basename(file))[0]
        data_path = os.path.join(args.output, file_name + ".npz")
        other_data = {
            "ee_traj": [],
            "joint_traj": [],
            "gripper": [],
            "est_force": [],
            "est_torque": [],
            "measured_force": [],
            "measured_torque": [],
            "compensated_force": [],
            "compensated_torque": [],
            "compensated_base_force": [],
            "compensated_base_torque": [],
            "zeroed_force": [],
            "zeroed_torque": [],
            "time": [],
        }

        for file_name, df in zip(csv_files, data_frames):
            if "Time" not in df.columns:
                continue
            time = df["Time"].values
            for index, row in df.iterrows():
                time = row["Time"]
                data = extract_data(file_name, row)
                if data is None:
                    continue
                if "ee_pose" in data.keys():
                    other_data["ee_traj"].append(data["ee_pose"])
                    other_data["joint_traj"].append(data["joints"])
                if "gripper" in data.keys():
                    other_data["gripper"].append(data["gripper"])
                if "est_force" in data.keys():
                    other_data["est_force"].append(data["est_force"])
                    other_data["est_torque"].append(data["est_torque"])
                if "measured_force" in data.keys():
                    other_data["measured_force"].append(data["measured_force"])
                    other_data["measured_torque"].append(data["measured_torque"])
                if "compensated_force" in data.keys():
                    other_data["compensated_force"].append(data["compensated_force"])
                    other_data["compensated_torque"].append(data["compensated_torque"])
                if "compensated_base_force" in data.keys():
                    other_data["compensated_base_force"].append(data["compensated_base_force"])
                    other_data["compensated_base_torque"].append(data["compensated_base_torque"])
                if "zeroed_force" in data.keys():
                    other_data["zeroed_force"].append(data["zeroed_force"])
                    other_data["zeroed_torque"].append(data["zeroed_torque"])
                other_data["time"].append(time)


        # import pdb
        # pdb.set_trace()

        # Aligns array sizes using minimum length
        non_empty_keys = [k for k in other_data.keys() if len(other_data[k]) > 0]
        if non_empty_keys:
            min_length = min(len(other_data[k]) for k in non_empty_keys)
            for k in non_empty_keys:
                other_data[k] = other_data[k][:min_length]

            # Converti in array NumPy
            for k, v in other_data.items():
                other_data[k] = np.array(v)
            if len(other_data["time"]) > 0:
                other_data["time"] = other_data["time"] - other_data["time"][0]

            # Initial position and orientation
            initial_position = other_data["ee_traj"][0][:3]
            initial_orientation = other_data["ee_traj"][0][3:]

            # Salva i dati come .npz
            np.savez(data_path, **other_data, initial_position=initial_position, initial_orientation=initial_orientation)

            data = np.load(data_path)
            print(f"Loaded data keys: {data.files}")
            print(f"ee_traj shape = {data['ee_traj'].shape}, joint_traj shape = {data['joint_traj'].shape}, gripper shape ={data['gripper'].shape} est_force shape = {data['est_force'].shape}, est_torque shape = {data['est_torque'].shape}, measured_force shape = {data['measured_force'].shape}, measured_torque shape = {data['measured_torque'].shape}, compensated_force shape = {data['compensated_force'].shape}, compensated_torque shape = {data['compensated_torque'].shape}, compensated_base_force shape = {data['compensated_base_force'].shape}, compensated_base_torque shape = {data['compensated_base_torque'].shape}, zeroed_force shape = {data['zeroed_force'].shape}, zeroed_torque shape = {data['zeroed_torque'].shape}, time shape = {data['time'].shape}")
            print(f"Saved data to {data_path}")
        else:
            print(f"No valid data found in {file}")
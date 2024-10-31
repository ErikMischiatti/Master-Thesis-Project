A basic ros package for segmenting recodings, encoding motions in cartesian dmps, and playing learned dmps in ROS.


## Usage
1. use `process_data.py` to segement the recording
2. use `dmp_learn.py` to encode the dmp
3. use `roslaunch dmp_ros dmp_play` to play the encoded motion specified in `dmp_play.yaml`

## Data recording
1. `roscore`
2. `roslaunch franka_example_controllers move_to_start.launch`
3. `roslaunch franka_interactive_controllers joint_gravity_compensation_controller.launch load_gripper:=true load_franka_control:=true robot_ip:=franka use_gripper_gui:=true`
4. `roslaunch dmp_ros record.launch`


## notes
roslaunch dmp_ros PlayDMP.launch use_sim:=false record:=false use_gripper:=true motion:=LNdF_pick_r_1_dmp

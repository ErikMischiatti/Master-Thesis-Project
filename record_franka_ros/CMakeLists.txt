cmake_minimum_required(VERSION 3.0.2)
project(record_franka_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(
  PROGRAMS
    scripts/get_ee_pose_vel.py
    scripts/buttons_ctrl.py
    # scripts/setup_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Bash scripts
install(PROGRAMS
  scripts/setup_camera.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

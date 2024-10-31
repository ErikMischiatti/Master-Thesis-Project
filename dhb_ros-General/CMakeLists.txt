cmake_minimum_required(VERSION 3.0.2)
project(dhb_ros)

## Trova i pacchetti richiesti, incluso actionlib_msgs per le Action
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  franka_msgs
  cartesian_control_msgs
  actionlib_msgs  # Aggiungi actionlib_msgs
  message_generation  # Aggiungi message_generation per la generazione dei messaggi
)

## Impostazioni per il pacchetto Python
catkin_python_setup()

## Definisci i messaggi action
add_action_files(
  FILES
  DHBAction.action  # Il file .action che definisce la tua Action
)

## Genera i messaggi e le action
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

## Dichiarazione del pacchetto
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs franka_msgs cartesian_control_msgs actionlib_msgs message_runtime  # Aggiungi message_runtime
)

## Includi le directory necessarie
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Trova tutti gli script Python
file(GLOB SCRIPTS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} scripts/*)
foreach(SCRIPT IN LISTS SCRIPTS)
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${SCRIPT})
        list(REMOVE_ITEM SCRIPTS ${SCRIPT})
    endif()
endforeach()

## Installa gli script Python
catkin_install_python(PROGRAMS ${SCRIPTS}
  scripts/dhb_play_trasl.py
  scripts/dhb_play.py
  scripts/computeDHB.py
  scripts/dhb_play_trasl_orient.py
  scripts/orientation_test.py
  scripts/dhb_action_server.py
  scripts/dhb_action_client.py
  # scripts/reconstructTrajectory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Installa la cartella launch
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

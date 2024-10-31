#!/usr/bin/env python3

import rospy
import actionlib
from dhb_ros.msg import DHBActionAction, DHBActionGoal
from geometry_msgs.msg import PoseStamped

class DHBActionClient:
    def __init__(self):
        # Inizializza il nodo ROS
        rospy.init_node('dhb_action_client')

        # Attributi per salvare la posizione e l'orientamento del cavo
        self.cable_pose = None
        self.orientation_cable = None

        # Sottoscrizione al topic /cable_pose
        rospy.Subscriber("/cable_pose", PoseStamped, self.cable_pose_cb)

        # Crea il client action
        self.client = actionlib.SimpleActionClient('pick', DHBActionAction)

        # Aspetta che il server sia pronto
        rospy.loginfo("In attesa del server di azioni 'pick'...")
        self.client.wait_for_server()
        rospy.loginfo("Il server di azioni 'pick' è pronto.")

        # Aspetta finché non viene ricevuta una posizione valida del cavo
        rospy.loginfo("In attesa della posizione del cavo...")
        while self.cable_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.5)

        # Una volta ricevuti i dati, invia il goal
        self.send_goal()

    def cable_pose_cb(self, msg):
        """Callback per aggiornare la posizione e l'orientamento del cavo."""
        self.cable_pose = msg.pose.position
        self.orientation_cable = msg.pose.orientation
        # rospy.loginfo(f"Posizione del cavo ricevuta: {self.cable_pose}")
        # rospy.loginfo(f"Orientamento del cavo ricevuto: {self.orientation_cable}")

    def send_goal(self):
        """Invia il goal al server di azioni."""
        # Crea e imposta il goal
        goal = DHBActionGoal()
        goal.slowdown_factor = 3.0  # Modifica questo valore se necessario

        # Imposta il goal usando la posizione e l'orientamento del cavo
        goal.goal.position.x = self.cable_pose.x
        goal.goal.position.y = self.cable_pose.y
        goal.goal.position.z = self.cable_pose.z
        goal.goal.orientation.x = self.orientation_cable.x
        goal.goal.orientation.y = self.orientation_cable.y
        goal.goal.orientation.z = self.orientation_cable.z
        goal.goal.orientation.w = self.orientation_cable.w

        rospy.loginfo(f"Inviando il goal con posizione: {self.cable_pose.x}, {self.cable_pose.y}, {self.cable_pose.z}")
        rospy.loginfo(f"E orientamento: {self.orientation_cable.x}, {self.orientation_cable.y}, {self.orientation_cable.z}, {self.orientation_cable.w}")

        # Invia il goal al server
        self.client.send_goal(goal)

        # Attende il risultato
        self.client.wait_for_result()
        result = self.client.get_result()

        # Verifica l'esecuzione del goal
        if result:
            rospy.loginfo("Goal completato con successo.")
        else:
            rospy.logerr("Esecuzione del goal fallita.")

if __name__ == '__main__':
    try:
        client = DHBActionClient()
    except rospy.ROSInterruptException:
        rospy.logerr("Programma interrotto prima del completamento")

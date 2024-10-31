import rospy
from std_msgs.msg import Int32, Bool
import subprocess
import os
import signal
import json
import hashlib
import base64
import ssl
import time
from http.client import HTTPSConnection, HTTPException
import threading 
from enum import Enum
import rospkg


class RecordManager:
    def __init__(self):
        # Recording states
        self.is_recording = False
        self.segment_num = 0
        self.recording_process = None

        # Button states
        self.prev_check = None
        self.prev_circle = None

        # Franka Web stuff
        self.hostname = '172.16.1.2'
        self.login = 'FrankaPanda1'
        self.password = 'ASLab2022'
        # HOSTNAME = '172.16.0.2'
        # LOGIN = 'FrankaPanda2'
        # PASSWORD = 'ASLab2022'

        # Paths
        # self.rospack = rospkg.RosPack()
        # self.rospack_path = self.rospack.get_path('record_teleop')

        rospy.init_node("record_manager", anonymous=True)

        # Pubs
        self.pub_record_flag = rospy.Publisher("/record_manager/record_flag", Bool, queue_size=1)
        self.pub_segment_num = rospy.Publisher("/record_manager/segment_num", Int32, queue_size=1)

        # Subs
        # rospy.Subscriber("/vr_buttons", Buttons, self.buttons_callback)
        self.buttons = FrankaWebInterface(self.hostname, self.login, self.password, self.callback)
        # self.buttons.start()
        rospy.loginfo("Initialized FrankaWebInterface")

        


    def start_recording(self):
        if not self.is_recording:
            rospy.loginfo("Starting recording...")
            # Get params
            topic = rospy.get_param("~topic", "/record_manager/record_flag /record_manager/segment_num")
            topic = topic.split()
            path_save = rospy.get_param("~path_save", "/home/asl_team/teleop_recordings")      # change this if necessary
            file_name = rospy.get_param("~file_name", "")

            # Start the recording node
            command = ["rosbag", "record",
                        "-o", os.path.join(path_save, file_name)]
            command.extend(topic)
            rospy.logwarn("Launching roslaunch with args: {}".format(command))
            rospy.logerr(" ".join(command))
            self.recording_process = subprocess.Popen(command)

            self.is_recording = True
            self.segment_num = 0
            rospy.loginfo("Started recording.")

    def stop_recording(self):
        if self.is_recording:
            rospy.loginfo("Stopping recording...")
            if self.recording_process:
                # Terminate the rosbag process
                self.recording_process.terminate()
                self.recording_process.wait()
                self.recording_process = None
            self.is_recording = False
            self.segment_num = 0
            rospy.loginfo("Stopped recording.")

    def callback(self, msg):
        if(list(msg.keys())[0] == "circle"):
            if list(msg.values())[0] and not self.is_recording and not self.prev_circle:
                self.start_recording()
                self.prev_circle = True
            elif list(msg.values())[0] and self.is_recording and not self.prev_circle:
                self.stop_recording()
                self.prev_circle = True
            elif not list(msg.values())[0]:
                self.prev_circle = False
        elif(list(msg.keys())[0] == "check"):
            if list(msg.values())[0] and self.is_recording and not self.prev_check:
                rospy.loginfo("Started new segmentation part.")
                self.segment_num += 1
                self.prev_check = True
            elif list(msg.values())[0] and not self.is_recording and not self.prev_check:
                rospy.loginfo("Currently not recording -> Can't start new segmentation part.")
            elif not list(msg.values())[0]:
                self.prev_check = False
        
        # Publish the flags
        self.pub_record_flag.publish(self.is_recording)         # pub_record_flag
        self.pub_segment_num.publish(self.segment_num)          # pub_segment_num
        


    # VR Buttons
    # def buttons_callback(self, msg: Buttons):                                
    #     # Handle menu -> menu button is used to flip 'record' flag
    #     if msg.menu.data and msg.menu.data != self.prev_menu:
    #         if not self.is_recording:
    #             self.start_recording()
    #         else:
    #             self.stop_recording()

    #     # Handle touchpad -> pressing touchpad adds a segmentation point 
    #     if msg.touchpad.data and msg.touchpad.data != self.prev_touchpad:
    #         if self.is_recording:
    #             rospy.loginfo("Started new segmentation part.")
    #             self.segment_num += 1
    #         else:
    #             rospy.loginfo("Currently not recording -> Can't start new segmentation part.")

    #     # Publish the flags
    #     self.pub_record_flag.publish(self.is_recording)       # pub_record_flag
    #     self.pub_segment_num.publish(self.segment_num)        # pub_segment_num

    #     # Remember prev state
    #     self.prev_menu = msg.menu.data
    #     self.prev_touchpad = msg.touchpad.data



class FrankaButtons(Enum):
    LEFT = "left"
    RIGHT = "right"
    UP = "up"
    DOWN = "down"

    CIRCLE = "circle"
    CROSS = "cross"
    CHECK = "check"

class FrankaWebInterface(threading.Thread):
    """API for using Franka methods available in Desk web interface. \
       Uses HTTPS connection and sends requests.

    init args:
        hostname (string): robot IP address used to connect to Franka Desk web interface.
        login (string): User login in Franka Desk web interface.
        password (string): User password in Franka Desk web interface
    """

    def __init__(self, hostname, user, password, callback=None):
        super().__init__()
        self._hostname = hostname
        self._user = user
        self._password = password

        self._client = HTTPSConnection(self._hostname, context=ssl._create_unverified_context())
        self._client.connect()
        encoded_password = self._encode_password(self._user, self._password)
        body_data = json.dumps({'login': self._user, 'password': encoded_password})
        self._client.request('POST', '/admin/api/login',
                             body=body_data,
                             headers={'content-type': 'application/json'})
        self._token = self._client.getresponse().read().decode('utf8')

        self._callback = callback

        self.stop = False

    def _encode_password(self, user, password):
        bs = ','.join([str(b) for b in hashlib.sha256((password + '#' + user + '@franka').encode('utf-8')).digest()])
        return base64.encodebytes(bs.encode('utf-8')).decode('utf-8')
    
    def run(self):
        while(True):
            if self.stop:
                break
            self._client.request("GET", "/desk/api/navigation/events", 
                                headers={'content-type': 'application/x-www-form-urlencoded',
                                'Cookie': 'authorization=%s' % self._token})
            button_data = self._client.getresponse().read()

            button_data = json.loads(button_data)

            if(self._callback is not None):
                self._callback(button_data)

        self._client.close()

    

if __name__ == "__main__":

    record_manager = RecordManager()
    record_manager.buttons.start()
    rospy.spin()




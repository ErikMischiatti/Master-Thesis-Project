import json
import hashlib
import base64
import ssl
import time
from http.client import HTTPSConnection, HTTPException
import threading 
from enum import Enum

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

        self._callback = callback

        self._client = None
        self._token = None
        self.stop = False

        self._connect()

    def _connect(self):
        try:
            self._client = HTTPSConnection(self._hostname, context=ssl._create_unverified_context())
            self._client.connect()
            encoded_password = self._encode_password(self._user, self._password)
            body_data = json.dumps({'login': self._user, 'password': encoded_password})
            self._client.request('POST', '/admin/api/login',
                                 body=body_data,
                                 headers={'content-type': 'application/json'})
            self._token = self._client.getresponse().read().decode('utf8')
        except HTTPException as e:
            print(f"Failed to connect: {e}")

    def _encode_password(self, user, password):
        bs = ','.join([str(b) for b in hashlib.sha256((password + '#' + user + '@franka').encode('utf-8')).digest()])
        return base64.encodebytes(bs.encode('utf-8')).decode('utf-8')
    
    def run(self):
        while True:
            if self.stop:
                break
            try:
                self._client.request("GET", "/desk/api/navigation/events",
                                     headers={'content-type': 'application/x-www-form-urlencoded',
                                              'Cookie': 'authorization=%s' % self._token})
                button_data = self._client.getresponse().read()
                button_data = json.loads(button_data)

                if self._callback is not None:
                    self._callback(button_data)
            except Exception as e:
                print(f"Failed to get button data: {e}")
                self._connect()

        self._client.close()

    
if __name__ == "__main__":
    import rospy
    from record_franka_ros.Gripper import Gripper
    from record_ros.srv import *

    rospy.init_node("gripper_buttons")
    print("wating for service ...")
    rospy.wait_for_service('/record/cmd')
    print("Got serivce")
    record_ros = rospy.ServiceProxy('/record/cmd', String_cmd)
    gripper = Gripper()

    recording = False
    opened = True

    # #  Franka 2
    # HOSTNAME = '172.16.0.2'
    # LOGIN = 'FrankaPanda2'
    # PASSWORD = 'ASLab2022'

    #  Franka 1
    HOSTNAME = '172.16.1.2'
    LOGIN = 'FrankaPanda1'
    PASSWORD = 'ASLab2022'

    def callback(msg):
        global gripper
        global opened
        global record_ros
        global recording

        # print(msg)
        try: 
            if(list(msg.keys())[0] == "left"):
                if list(msg.values())[0] and not recording:
                    recording = True
                    record_ros("record")
                    print("start recording")
            elif(list(msg.keys())[0] == "right"):
                if list(msg.values())[0] and recording:
                    recording = False
                    record_ros("stop")
                    print("stop recording")
            elif(list(msg.keys())[0] == "up"):
                if list(msg.values())[0] and not opened:
                    opened = True
                    print("open")
                    gripper.open()
            elif(list(msg.keys())[0] == "down"):
                if list(msg.values())[0] and opened:
                    opened = False
                    print("close")
                    gripper.close()
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    button = FrankaWebInterface(HOSTNAME, LOGIN, PASSWORD, callback=callback)
    button.start()
    rospy.spin()
from geometry_msgs.msg import Pose, PoseStamped

def create_pose(x, y, z, qx, qy, qz, qw):
    msg = Pose()
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z

    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw

    res = PoseStamped()
    res.pose = msg

    return res
import tf
from math import pi
from geometry_msgs.msg import Quaternion


def yaw2quaternion(yaw):

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw/360.0*2*pi)
    q = Quaternion()

    q.x = quaternion[0]
    q.y = quaternion[1]
    q.z = quaternion[2]
    q.w = quaternion[3]

    return q


def quaternion2yaw(q):

        quaternion = (
            q.x,
            q.y,
            q.z,
            q.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]/(2*pi)*360


def get_pick_polygon_points(msg):

    poly_points = []

    for pt in msg.polygon[0].polygon.points:

        poly_points.append((pt.x, pt.y))

    return poly_points

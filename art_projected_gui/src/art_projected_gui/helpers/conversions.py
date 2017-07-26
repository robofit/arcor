import tf
from math import pi, sqrt
from geometry_msgs.msg import Quaternion


def is_close(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


def pos2str(pos):

    return "[X: " + str(round(pos[0], 3)).ljust(5, '0') + ", Y: " + str(round(pos[1], 3)).ljust(5, '0') + ", Z: " + str(round(pos[2], 3)).ljust(5, '0') + "]"


def q2a(q):

    return (
        q.x,
        q.y,
        q.z,
        q.w)


def a2q(arr):

    q = Quaternion()

    q.x = arr[0]
    q.y = arr[1]
    q.z = arr[2]
    q.w = arr[3]

    return q

# rotate vector v1 by quaternion q1


def qv_mult(q1, v1):
    v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2),
        tf.transformations.quaternion_conjugate(q1)
    )[:3]


def yaw2quaternion(yaw):

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw / 360.0 * 2 * pi)

    return a2q(quaternion)


def quaternion2yaw(q):

    quaternion = (
        q.x,
        q.y,
        q.z,
        q.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler[2] / (2 * pi) * 360


def quaternion2rpy(q):

    quaternion = (
        q.x,
        q.y,
        q.z,
        q.w)

    euler = list(tf.transformations.euler_from_quaternion(quaternion))

    for idx in range(0, len(euler)):

        euler[idx] = euler[idx] / (2 * pi) * 360

        if euler[idx] > 180:
            euler[idx] -= 180

    return euler


def get_pick_polygon_points(polygon_arr):

    poly_points = []

    for pt in polygon_arr[0].polygon.points:

        poly_points.append((pt.x, pt.y))

    return poly_points

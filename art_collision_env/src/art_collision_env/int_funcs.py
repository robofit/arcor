from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker, Marker


def axis_control(name, q):

    cr = InteractiveMarkerControl()
    cr.orientation.x = q[0]
    cr.orientation.y = q[1]
    cr.orientation.z = q[2]
    cr.orientation.w = q[3]
    cr.name = name
    return cr


def rotate_axis_control(name, q):

    cr = axis_control(name, q)
    cr.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    return cr


def move_axis_control(name, q):

    cr = axis_control(name, q)
    cr.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    return cr


def make_def(p, color=(0.5, 0.5, 0.5), desc=None):

    im = InteractiveMarker()
    im.header.frame_id = p.pose.header.frame_id
    im.pose = p.pose.pose
    im.name = p.name
    if desc is None:
        im.description = p.name
    im.scale = 1.2 * max(p.bbox.dimensions)

    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = p.bbox.dimensions[0]
    marker.scale.y = p.bbox.dimensions[1]
    marker.scale.z = p.bbox.dimensions[2]
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0

    control = InteractiveMarkerControl()
    control.always_visible = True
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.markers.append(marker)
    im.controls.append(control)

    return im

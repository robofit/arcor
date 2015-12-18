# art_umf_localizer

ROS wrapper for UMF detector library. It publishes TF transform between marker (parameter world_frame, i.e. "marker") and robot (parameter robot_frame, i.e. "odom_combined"). It can do so continuously or on request (actionlib interface). When used in "on-demand" mode it performs marker detection and then publishes the same transform until next request.

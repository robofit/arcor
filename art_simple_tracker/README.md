Simple test:

````
rosrun tf static_transform_publisher 0.5 0 0 0 0 0 marker kinect_1 10
rosrun tf static_transform_publisher -2 0 0 0 0 0 marker kinect_2 10
rosrun art_simple_tracker fake_detector.py 21 kinect_1 -0.5 0 0 0.05
rosrun art_simple_tracker fake_detector.py 21 kinect_2 1.5 0 0 0.3
rosrun art_simple_tracker tracker.py
rostopic echo /art/object_detector/object_filtered
````
# Installation
## Build v4r library from source

Firstly install the dependencies for the v4r library:
 - openni drivers: sudo apt-get install libopenni-dev libopenni-sensor-primesense0
 - Qt 4 (http://qt-project.org/): sudo apt-get install libqt4-dev
 - Boost (http://www.boost.org/): comes with ubuntu
 - Point Cloud Library 1.7.x (http://pointclouds.org/): comes with ROS
 - Eigen3 (http://eigen.tuxfamily.org/): sudo apt-get install libeigen3-dev
 - OpenCV 2.x (http://opencv.org/): comes with ROS
 - Ceres Solver 1.9.0 (http://ceres-solver.org/): sudo apt-get install libceres-dev
 - OpenGL GLSL mathematics (libglm-dev): sudo apt-get install libglm-dev
 - GLEW - The OpenGL Extension Wrangler Library (libglew-dev): sudo apt-get install libglew-dev
 - libraries for sparse matrices computations (libsuitesparse-dev): sudo apt-get install libsuitesparse-dev

Run commands:

cd ~/somewhere

git clone 'https://github.com/strands-project/v4r'

cd v4r

mkdir build && cd build

cmake ..

make

sudo make install


## Install ROS wrapper

Put the v4r_ros_wrappers, art_object_recognizer_msgs and v4r_recognizer_launch directories to your catkin workspace.
Put the directory 'data' to your home directory.

cd ~/catkin_ws

catkin_make


# Using object recognizer
1. Run kinect2_bridge
2. Run launcher for v4r recognizer:

roslaunch v4r_recognizer_launch recognizer.launch

3. Run rviz for visualization results:

rosrun rviz rviz

4. Display recognition results:

rostopic echo /recognition_service/obj_instances_poses

## Visualizing results
The model for recognized object will be published to the topic /recognition_service/sv_recogniced_object_instances

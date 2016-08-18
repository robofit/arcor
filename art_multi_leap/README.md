The node (```leap_client.py```) reads data from one or more LeapMotion sensors using websocket interface. It is supposed that sensors are placed on a table in front of the user. The node publishes following messages (at 30 Hz):

- ```/art/user/palm/3d/left (geometry_msgs/PoseStamped)```
- ```/art/user/palm/3d/right (geometry_msgs/PoseStamped)```
- ```/art/user/palm/2d/left (geometry_msgs/PointStamped)```
- ```/art/user/palm/2d/right (geometry_msgs/PointStamped)```

...where "3d" topics are user's palm poses (for left and right hand) and "2d" topics are x,y points on the table where user points. Table size can be specified by parameters (see launch file for example). Sensors can be specified in yaml file (see config/leaps.yml). One can specify position and orientation (orientation is ignored for now) of each device  and then, when there is overlap, data from two or more sensors are averaged using confidence values as weights. Moreover, there is simple adjustable temporal filtering.

As one can only attach one LeapMotion device to one PC, other devices have to be attached to another PCs or to virtual machines. As ```leapd``` only accepts connections from the same IP, ports should be forwarded like this:

```ssh -L 9000:localhost:6437 user@another_machine_with_leap_connected```

Dependencies:

```
sudo pip install pyrr ws4py numpy
```

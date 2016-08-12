The node (```leap_client.py```) reads data from one or more LeapMotion sensors using websocket interface and (so far) publishes left and right palm positions as ```geometry_msgs/PoseStamped``` messages at 30 Hz. One can specify position and orientation (orientation is ignored for now) of each device (so far only by editing the code) code and then, when there is overlap, data from two or more sensors are averaged using confidence values as weights. Moreover, there is simple adjustable temporal filtering of the palm position.

As one can only attach one LeapMotion device to one PC, other devices have to be attached to another PCs or to virtual machines. As ```leapd``` only accepts connections from the same IP, ports should be forwarded like this:

```ssh -L 9000:localhost:6437 user@another_machine_with_leap_connected```

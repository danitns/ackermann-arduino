# ackermann_arduino

_CarlikeBot_, or ''Carlike Mobile Robot'', is a simple mobile base with ackermann drive.
It is expected to communicate via serial with an Arduino.
The robot has two wheels in the front that steer the robot and two wheels in the back that power the robot forward and backwards. However, since the front pair of wheels (steering) are controlled by one interface, a ackermann steering model is used.

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_11/doc/userdoc.html).

It is based on the bicycle example from [ros2_control demos](https://github.com/ros-controls/ros2_control_demos/tree/master/example_11).

Useful resource for developing a hardware interface like this:
https://youtu.be/J02jEKawE5U

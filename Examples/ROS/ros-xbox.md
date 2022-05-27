# ROS Wired Xbox Controller Integration
Details about joystick_drivers ROS package, used to interface
with an xbox controller for manual control of the robot. 

Reference: http://wiki.ros.org/joy?distro=noetic

## Installation 
```
sudo apt-get install ros-noetic-joy
```

## Usage 
The `joy` sub-package contains a node called `joy_node`. This
node publishes the message `joy` which contains the state of each
button / joystick on the gamepad.

Sample node execution
```
rosrun joy joy_node _dev:="/dev/input/js1"
```

## Configuration
The node provides a set of ROS parameters used to specify information 
about the device in use. 

### Parameters: 

| Name              |  Type   | Default           |
|:-----------------:|:-------:|:-----------------:|
| dev               | str     |  "/dev/input/jso" |
| deadzone          | double  | 0.05              |
| autorepeat_rate   | double  | 0.0 (disabled)    |
| coalesce_interval | double  | 0.001             |
| default_trig_val  | bool    | false             |

Note: (All are private, i.e. should be prefixed with `_` to remap from commandline as seen in previous section)

## Table of index number of /joy.buttons:

0 - A

1 - B

2 - X

3 - Y

4 - LB
 
5 - RB
 
6 - back
 
7 - start
 
8 - power
 
9 - Button stick left
 
10 -  Button stick right

## Table of index number of /joy.axes:

0 - Left/Right Axis stick left

1 - Up/Down Axis stick left

2 - Left Trigger

3 - Left/Right Axis stick right

4 - Up/Down Axis stick right

5 - Right Trigger

6 - cross key left/right

7 - cross key up/down

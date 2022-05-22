# ROS-CAN 

## socketcan_bridge_node
Spin up the node: 
```
rosrun socketcan_bridge socketcan_bridge_node _can_device:=vcan0
```

Interact with device parameter
```
rosparam set /socketcan_bridge_node/can_device "device"
rosparam get /socketcan_bridge_node/can_device
```

This will publish all CAN messages received on the specified CAN device to
the topic `/received_messages`. The messages are of type `can_msgs/Frame`
and are stuctured as follows:
```
header: 
  seq: 560
  stamp: 
    secs: 1653066255
    nsecs: 550524554
  frame_id: ''
id: 5
is_rtr: False
is_extended: False
is_error: False
dlc: 3
data: [5, 6, 7, 78, 28, 37, 16, 82]
```

To view the messages as they are published:
```
rostopic echo /received_messages
```

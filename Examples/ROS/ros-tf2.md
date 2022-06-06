# ROS tf2 package
using ROS transforms to convert between various local coordinate spaces (frames).

## Static publisher
A static transform between parent coordinate frame and child coordinate frame can be published using: 

Using rosrun
```
rosrun tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
```

Using launch file: 
```
<launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="1 0 0 0 0 0 1 link1_parent link1" />
</launch>
```

## BEV Transform 
```
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 zed2_camera_center bev
```
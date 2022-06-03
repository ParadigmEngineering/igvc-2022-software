# ROS GPS Integration 
We are using an ardusimple SimpleRTK2b evaluation board with a ublox ZED-F9P high 
precision GNSS module. 

## NMEA Over Serial 
The GPS module is connected to the Control Laptop via USB. Using the
u-blox u-center application, the board can be configured to output basic
NMEA strings over serial, which can be parsed to obtain GNSS readings. 

## ROS nmea-navsat-driver
Refer: https://wiki.ros.org/nmea_navsat_driver

The ROS pacakge `nmea-navsat-driver` can be used to listen over serial for NMEA strings,
and publish GNSS readings as ROS message of type: 

## Package Installation
For ros noetifc: 
```
sudo apt-get install ros-noetic-nmea-navsat-driver
```

## Sample Usage
To get up and running quickly:
```
rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyUSB0 _baud:=38400

Generally:
rosrun c nmea_serial_driver _port:=/dev/ttyACM0 _baud:=38400
```

This will publish data to the following topics: 
```
fix: 
    desc: GPS position fix reported by the device
    type: sensor_msgs/NavSatFix
    
vel: 
    desc: velocity output from the GPS device 
    type: geometry_msgs/TwistStamped

time_reference: timestamp from the GPS device
    desc: GPS position fix reported by the device
    type: sensor_msgs/TimeReference
```

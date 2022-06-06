# CANABLE USB <-> CAN Connector

Install can utils and network utils
```
sudo apt install can-utils
sudo apt install net-utils
```

Configure can 
Note: replace /dev/ttyACM0 with the proper device, check with `dmesg`
```
sudo slcand -o -c -s0 /dev/ttyACM0 can0
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000
```

CAN Utils
```
cansend can0 999#DEADBEEF   # Send a frame to 0x999 with payload 0xdeadbeef
candump can0                # Show all traffic received by can0
canbusload can0 500000      # Calculate bus loading percentage on can0 
cansniffer can0             # Display top-style view of can traffic
```
""" can_sender.py

Example of sending a CAN message over ROS using socketcan_bridge node. 
Sent messages can be monitored using the can_logger node.
"""

import rospy
from can_msgs.msg import Frame

def sender():
    pub = rospy.Publisher("sent_messages", Frame, queue_size=10)
    rospy.init_node("can_sender", anonymous=True)
    rospy.loginfo("Sending example CAN frame at 1 Hz...")

    rate = rospy.Rate(1) # 1Hz
    while not rospy.is_shutdown():
        frame = Frame()
        frame.header.stamp = rospy.Time.now()
        
        frame.id = 42
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 8
        frame.data = [1, 2, 3, 4, 5, 6, 7, 8]
        
        log_message = f"ID: {frame.id:<3} DLC: {frame.dlc: <1} Data: {frame.data}"
        rospy.loginfo(log_message)

        pub.publish(frame)
        rate.sleep()

if __name__ == "__main__":
    try:
        sender()
    except rospy.ROSInterruptException:
        pass

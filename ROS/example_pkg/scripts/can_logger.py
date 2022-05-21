""" can_logger.py

Listen for and log CAN frames published by socketcan_bridge.  
"""

import rospy
from can_msgs.msg import Frame

def handle_frame(frame: Frame):
    """ Log received CAN frame 
    
    frame.id = CAN Arbitration ID 
    frame.is_rtr = Remote transmission request
    frame.is_extended = If true, is a CANFD / extended CAN frame
    frame.is_error = Frame is in error (CRC)

    frame.dlc = Data length code (# bytes)
    frame.data = Data (always uint8[8], only read up to DLC bytes)
    """
    data = []
    for i in range(frame.dlc):
        data.append(frame.data[i])
    
    log_message = f"ID: {frame.id:<3} DLC: {frame.dlc: <1} Data: {data}"
    rospy.loginfo(log_message)

def listener():
    rospy.init_node("can_logger_node", anonymous=True, log_level=rospy.INFO)
    rospy.loginfo("Waiting for CAN messages...")
    rospy.Subscriber("received_messages", Frame, handle_frame)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

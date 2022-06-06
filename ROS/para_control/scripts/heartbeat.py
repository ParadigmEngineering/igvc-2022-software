import rospy
from can_msgs.msg import Frame
from para_control.frog_can_commands import gen_heartbeat_command


def main():
    rospy.init_node("heartbeat", log_level=rospy.DEBUG)
    can_pub = rospy.Publisher("sent_messages", Frame, queue_size=10)
    rospy.loginfo("Sending heartbeat CAN frame at 2 Hz...")

    rate = rospy.Rate(5) # 5 Hz
    frame = gen_heartbeat_command()
   
    while not rospy.is_shutdown():
        rospy.loginfo(f"ID: {frame.id}")
        can_pub.publish(frame)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
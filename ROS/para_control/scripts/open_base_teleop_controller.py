""" open_base_teleop_controller.py 

Listen for Xbox controller messages and emit open base control commands. 
"""

import rospy
from sensor_msgs.msg import Joy


def joy_to_sim_command(controller_state: Joy):
    pass


def main():
    rospy.init_node("open_base_teleop_controller", anonymous=True, log_level=rospy.INFO)
    rospy.loginfo("Waiting for controller updates...")
    rospy.Subscriber("/para/joy", Joy, joy_to_sim_command)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

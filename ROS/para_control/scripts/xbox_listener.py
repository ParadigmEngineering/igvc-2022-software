#!/usr/bin/env python3
""" joy_to_motor_command.py

Listen for /para/joy messages, and convert to motor commands. 
"""

import rospy

from sensor_msgs.msg import Joy
from para_control.xbox_utils import XboxControllerState


controller_state = XboxControllerState()

def handle_joy_update(state: Joy):
    """ Convert joy stick state to a motor command 

    Left Stick X/Y -> [-1,1]
    Left/Right Trigger -> [-1 (squeezed), 1 (released)]
    """
    global controller_state
    controller_state.update_state_from_joy(state)
    rospy.loginfo \
    (
        f"LeftX - {controller_state.left_stick_x} | "
        f"LeftY - {controller_state.left_stick_y} | " 
        f"LeftTrigger - {controller_state.left_trigger} | "
        f"RightTrigger - {controller_state.right_trigger} | "
        f"Dpad X - {controller_state.dpad_x} | "
        f"Dpad Y - {controller_state.dpad_y} | "
    )


def main():
    """ Spin up node, subscribe to controller updates """
    rospy.init_node("xbox_listener", anonymous=True, log_level=rospy.INFO)
    rospy.loginfo("Waiting for controller updates...")
    rospy.Subscriber("/para/joy", Joy, handle_joy_update)
    rospy.spin()


if __name__ == "__main__":
    main()

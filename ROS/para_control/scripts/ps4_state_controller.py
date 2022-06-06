#!/usr/bin/env python3
""" ps4_state_controller.py

Listen for /para/joy messages, and convert to bot state commands. 
"""

import rospy

from sensor_msgs.msg import Joy
from can_msgs.msg import Frame
from para_control.ps4_utils import PS4ControllerState
from para_control.frog_can_commands import BOT_STATE, gen_state_command


controller_state = PS4ControllerState()
can_pub = rospy.Publisher("sent_messages", Frame, queue_size=10)


def handle_joy_update(state: Joy):
    """ Convert ps4 controller state to a state command

    Left Stick X/Y -> [-1,1]
    Left/Right Trigger -> [-1 (squeezed), 1 (released)]
    """
    # Update controller state
    global controller_state
    controller_state.update_state_from_joy(state)
    
    # Generate desired state 
    if not controller_state.left_bumper:
        return

    desired_state = None
    if controller_state.options:
        desired_state = BOT_STATE.STANDBY
    elif controller_state.triangle:
        desired_state = BOT_STATE.AUTONOMOUS
    elif controller_state.circle:
        desired_state = BOT_STATE.MANUAL
    else:
        # Return if no proper key combination 
        return

    can_frame = gen_state_command(desired_state)
    rospy.loginfo("Sending state command from ps4 controller...")
    rospy.logdebug(f"Desired state: {desired_state.name}")
    rospy.logdebug(f"State message CAN Frame: {can_frame}")

    can_pub.publish(can_frame)


def main():
    """ Spin up node, subscribe to controller updates """
    rospy.init_node("ps4_state_controller", log_level=rospy.DEBUG)
    rospy.loginfo("Waiting for controller updates...")
    rospy.Subscriber("/para/joy", Joy, handle_joy_update)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

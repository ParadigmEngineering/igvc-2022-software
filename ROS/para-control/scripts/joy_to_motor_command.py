""" joy_to_motor_command.py

Listen for /para/joy messages, and convert to motor commands. 
"""

import struct
import rospy

from enum import IntEnum, unique
from sensor_msgs.msg import Joy


@unique
class ControllerAxisIndices(IntEnum):
    """ XBOX Controller `Joy` message axis array indices
    """
    LEFT_X          = 0
    LEFT_Y          = 1
    LEFT_TRIGGER    = 2
    RIGHT_X         = 3
    RIGHT_Y         = 4
    RIGHT_TRIGGER   = 5
    DPAD_X          = 6
    DPAD_Y          = 7


@unique
class ControllerButtonIndices(IntEnum):
    """ Xbox Controller 'Joy' message button array indices
    """
    A                   = 0
    B                   = 1
    X                   = 2
    Y                   = 3
    LB                  = 4
    RB                  = 5
    BACK                = 6
    START               = 7
    POWER               = 8
    LEFT_STICK_CLICK    = 9
    RIGHT_STICK_CLICK   = 10


def joy_to_motor_command(state: Joy):
    """ Convert joy stick state to a motor command 

    Left Stick X/Y -> [-1,1]
    Left/Right Trigger -> [-1 (squeezed), 1 (released)]
    """
    left_stick_x = state.axes[ControllerAxisIndices.LEFT_X]
    left_stick_y = state.axes[ControllerAxisIndices.LEFT_Y]
    left_trigger = state.axes[ControllerAxisIndices.LEFT_TRIGGER]
    right_trigger = state.axes[ControllerAxisIndices.RIGHT_TRIGGER]

    rospy.loginfo("Controller updated")
    rospy.loginfo(
        f"LeftX - {left_stick_x} | "
        f"LeftY - {left_stick_y} | " 
        f"LeftTrigger - {left_trigger} | "
        f"RightTrigger - {right_trigger}"
    )


def main():
    """ Spin up node, subscribe to controller updates """
    rospy.init_node("joy_to_motor_command", anonymous=True, log_level=rospy.INFO)
    rospy.loginfo("Waiting for controller updates...")
    rospy.Subscriber("/para/joy", Joy, joy_to_motor_command)
    rospy.spin()


if __name__ == "__main__":
    main()

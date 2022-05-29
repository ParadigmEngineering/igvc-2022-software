""" open_base_teleop.py

Listen for key presses and convert to open_base commands. 
"""

import rospy
import sys, termios, tty

from open_base.msg import Movement, MovementGeneric
from enum import IntEnum, unique

# TODO: On node close, send velocity 0 command
# TODO: Implement press and hold 

@unique
class GenericMovementType(IntEnum):
    ABSOLUTE = 0
    RELATIVE = 1
    VELOCITY = 2


@unique
class GenericMovementFrame(IntEnum):
    HYBRID      = 0
    MOBILE      = 1
    RAW_MOBILE  = 2
    WORLD       = 3


move_bindings = \
{
	'w': [0, 1],
	'a': [-1, 0],
	's': [0, -1],
	'd': [1, 0],
    'q': [-1, 1],
    'e': [1, 1],
    'x': [0, 0]
}


speed_bindings = \
{
	'o': 1.1,
	'l': 0.9,
}


def get_key(): 
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)

    # TODO: Is settings here appropriate? 
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node("open_base_teleop", anonymous=True, log_level=rospy.INFO)
    command_pub = rospy.Publisher("/open_base/command", Movement, queue_size=1)

    rospy.loginfo("Starting open base keyboard teleop node...")

    command = Movement()
    command.movement = command.GENERIC

    command.generic.type = GenericMovementType.VELOCITY
    command.generic.frame = GenericMovementFrame.MOBILE
    command.generic.target.x = 0
    command.generic.target.y = 0
    command.generic.target.theta = 0

    max_speed = 1
    vel = [0, 0]

    while (1):
        key = get_key()

        if key in move_bindings.keys():
            vel = move_bindings[key]

        elif key in speed_bindings.keys():
            max_speed = max_speed * speed_bindings[key]
            max_speed = max(max_speed, 0.1)
            
        else: 
            vel = [0, 0]
            if (key == '\x03'):
                rospy.loginfo("Keyboard interrupt! Exiting...")
                break

        # TODO: Normalize direction vector first (diagonals are currently too fast)
        command_vel = [0, 0]
        command_vel[0] = vel[0] * max_speed
        command_vel[1] = vel[1] * max_speed

        rospy.loginfo(f"VEL_X: {command_vel[0]} | VEL_Y: {command_vel[1]}")
        rospy.loginfo(f"MAX_SPEED: {max_speed}")

        # Publish command in Open Base format
        command.generic.target.x = command_vel[0]
        command.generic.target.y = command_vel[1]
        command.generic.target.theta = 0

        command_pub.publish(command)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    try:
        main()
    except rospy.ROSInterruptException:
        pass

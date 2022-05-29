""" open_base_teleop.py

Listen for key presses and convert to open_base commands. 
"""

import rospy
import sys, termios, tty
from open_base.msg import Movement, MovementGeneric, Velocity


move_bindings = \
{
	'w': [0, 1],
	'a': [-1, 0],
	's': [0, -1],
	'd': [1, 0],
    'q': [-1, 1],
    'e': [1, 1]
}


speed_bindings = \
{
	'q': 1,
	'z': -1,
}


def get_key(): 
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)

    # TODO: Is settings here appropriate? 
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node("open_base_teleop", anonymous=True, log_level=rospy.INFO)
    rospy.loginfo("Starting open base keyboard teleop node...")

    max_speed = 1
    vel = [0, 0]

    while (1):
        key = get_key()

        if key in move_bindings.keys():
            vel = move_bindings[key]

        elif key in speed_bindings.keys():
            max_speed += speed_bindings[key]

        else: 
            vel = [0, 0]
            if (key == '\x03'):
                rospy.loginfo("Keyboard interrupt! Exiting...")
                break

        vel[0] = vel[0] * max_speed
        vel[1] = vel[1] * max_speed

        rospy.loginfo(f"VEL_X: {vel[0]} | VEL_Y: {vel[1]}")

        # TODO: Publish velocity command in open_base format


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    try:
        main()
    except rospy.ROSInterruptException:
        pass

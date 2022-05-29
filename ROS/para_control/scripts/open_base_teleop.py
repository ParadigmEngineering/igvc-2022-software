""" open_base_teleop.py

Listen for key presses and convert to open_base commands. 
"""

import rospy
import sys, termios, tty


move_bindings = \
{
	"W": (0, 1),
	"A": (-1, 0),
	"S": (0, -1),
	"D": (1, 0),
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
    rospy.loginfo("Starting open base keyobaord teleop node...")


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    main()

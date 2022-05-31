""" open_base_teleop_controller.py 

Listen for Xbox controller messages and emit open base control commands. 
"""

import rospy
from sensor_msgs.msg import Joy
from para_control.xbox_utils import XboxControllerState
from para_control.open_base_utils import GenericMovementFrame, GenericMovementType
from open_base.msg import Movement, MovementGeneric

state = XboxControllerState()
command_pub = rospy.Publisher("/open_base/command", Movement, queue_size=1)
max_speed = 1


def inc_max_speed():
    global max_speed
    max_speed = max_speed + (max_speed * 0.1) 


def dec_max_speed():
    global max_speed 
    max_speed = max_speed - (max_speed * 0.1) 


def joy_to_sim_command(controller_state: Joy):
    """ Recieve controller update, generate sim command """
    global max_speed
    global command_pub

    state.update_state_from_joy(controller_state)
    
    rotate = state.left_trigger != 1.0 or state.right_trigger != 1.0
    
    # Select mode of operation, give precedence to spot rotation
    if (rotate): 
        wheel_speed = state.right_trigger - state.left_trigger
        wheel_speed = wheel_speed * max_speed

        command = Movement()
        command.movement = command.WHEEL
        command.wheel.v_back = wheel_speed
        command.wheel.v_left = wheel_speed
        command.wheel.v_right = wheel_speed
        command_pub.publish(command)

    else:
        command = Movement()
        command.movement = command.GENERIC

        command.generic.type = GenericMovementType.VELOCITY
        command.generic.frame = GenericMovementFrame.MOBILE

        # Open base simulator positive x is right, xbox controller positive x is left 
        command.generic.target.x = -1 * max_speed * state.left_stick_x 
        command.generic.target.y = max_speed * state.left_stick_y
        command.generic.target.theta = 0
        command_pub.publish(command)

    # Handle max speed increases
    if state.dpad_y == 1:
        rospy.loginfo(f"Increased max speed: {max_speed}")
        inc_max_speed()
    
    elif state.dpad_y == -1:
        rospy.loginfo(f"Decreased max speed: {max_speed}")
        dec_max_speed()


def main():
    """ Spin up node, subscribe to joystick updated """
    rospy.init_node("open_base_teleop_controller", anonymous=True, log_level=rospy.INFO)
    rospy.loginfo("Waiting for controller updates...")
    rospy.Subscriber("/para/joy", Joy, joy_to_sim_command)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

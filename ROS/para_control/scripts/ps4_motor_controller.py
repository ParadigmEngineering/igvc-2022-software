#!/usr/bin/env python3
""" ps4_motor_controller.py

Listen for /para/joy messages, convert to motor command CAN messages. 
"""

import rospy
import numpy as np

from sensor_msgs.msg import Joy
from open_base.msg import Movement
from para_control.ps4_utils import PS4ControllerState
from para_control.frog_can_commands import *

controller_state = PS4ControllerState()
open_base_pub = rospy.Publisher("/open_base/command", Movement, queue_size=10)
can_pub = rospy.Publisher("sent_messages", Frame, queue_size=10)

# Max bot speed / 5 mi/hr
max_current = 100

# Wheel radius: 8 in = 0.2302 m
wheel_radius = 0.2032

# Bot radius (m) 
bot_radius = 0.3536


def vector_to_wheel_speed(vx: float, vy: float, theta_rad: float, wheel_radius: float, bot_radius: float):
    sinpi3 = 0.86602
    wheel_x = (-vx + bot_radius * theta_rad) / wheel_radius
    wheel_y = (-sinpi3 * vy + 0.5 *  (vx) + bot_radius * theta_rad) / wheel_radius
    wheel_z = (sinpi3 * vy + 0.5 * (vx) + bot_radius * theta_rad) / wheel_radius

    return [wheel_x, wheel_y, wheel_z]


def handle_joy_update(state: Joy):
    """ Convert joy stick state to a motor command 

    Left Stick X/Y -> [-1,1]
    Left/Right Trigger -> [-1 (squeezed), 1 (released)]
    """
    # Update controller state
    global controller_state
    controller_state.update_state_from_joy(state)

    global max_speed
    global open_base_pub
    global max_rot_speed
    
    rotate = controller_state.left_trigger != 1.0 or controller_state.right_trigger != 1.0
    
    # Select mode of operation, give precedence to spot rotation
    if (rotate): 
        local_max = 20
        current = controller_state.right_trigger - controller_state.left_trigger
        current = current * local_max
         
        print(f"CURRENT_BACK: {current}")
        print(f"CURRENT_LEFT: {current}")
        print(f"CURRENT_RIGHT: {current}\n")

        frame = gen_wheel_current_command(MODE_MASK_CURRENT.MANUAL, MOTOR_ID_MASK.MOTOR_BACK, current)
        can_pub.publish(frame)

        frame = gen_wheel_current_command(MODE_MASK_CURRENT.MANUAL, MOTOR_ID_MASK.MOTOR_LEFT, current)
        can_pub.publish(frame)

        frame = gen_wheel_current_command(MODE_MASK_CURRENT.MANUAL, MOTOR_ID_MASK.MOTOR_RIGHT, current)
        can_pub.publish(frame)

    else:
        vel_x = controller_state.left_stick_x
        vel_y = controller_state.left_stick_y

        wheel_speed = vector_to_wheel_speed(vel_x, vel_y, 0, wheel_radius, bot_radius)
        sum = 0
        for num in wheel_speed:
            sum = sum + num ** 2

        len = np.sqrt(sum)


        wheel_currents = []
        for speed in wheel_speed:
            wheel_currents.append((speed / max(len, 0.1)) * max_current)
            
        back_wheel_current = wheel_currents[0]
        left_wheel_current = wheel_currents[1]
        right_wheel_current = wheel_currents[2]

        print(f"CURRENT_BACK: {back_wheel_current}")
        print(f"CURRENT_LEFT: {left_wheel_current}")
        print(f"CURRENT_RIGHT: {right_wheel_current}\n")

        frame = gen_wheel_current_command(MODE_MASK_CURRENT.MANUAL, MOTOR_ID_MASK.MOTOR_BACK, back_wheel_current)
        can_pub.publish(frame)

        frame = gen_wheel_current_command(MODE_MASK_CURRENT.MANUAL, MOTOR_ID_MASK.MOTOR_LEFT, left_wheel_current)
        can_pub.publish(frame)

        frame = gen_wheel_current_command(MODE_MASK_CURRENT.MANUAL, MOTOR_ID_MASK.MOTOR_RIGHT, right_wheel_current)
        can_pub.publish(frame)


def main():
    """ Spin up node, subscribe to controller updates """
    rospy.init_node("ps4_motor_controller", log_level=rospy.DEBUG)
    rospy.loginfo("Waiting for controller updates...")
    rospy.Subscriber("joy", Joy, handle_joy_update)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

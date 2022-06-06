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
max_speed = 2.2352 

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
        wheel_speed = controller_state.right_trigger - controller_state.left_trigger
        wheel_speed = wheel_speed * max_speed
        wheel_rpm = (60 * wheel_speed) / (3.14 * 2 * wheel_radius)
         
        print(f"RPM_BACK: {wheel_rpm}")
        print(f"RPM_LEFT: {wheel_rpm}")
        print(f"RPM_RIGHT: {wheel_rpm}\n")

        frame = gen_wheel_rpm_command(MODE_MASK_RPM.MANUAL, MOTOR_ID_MASK.MOTOR_BACK, wheel_rpm)
        can_pub.publish(frame)

        frame = gen_wheel_rpm_command(MODE_MASK_RPM.MANUAL, MOTOR_ID_MASK.MOTOR_LEFT, wheel_rpm)
        can_pub.publish(frame)

        frame = gen_wheel_rpm_command(MODE_MASK_RPM.MANUAL, MOTOR_ID_MASK.MOTOR_RIGHT, wheel_rpm)
        can_pub.publish(frame)

    else:
        vel_x = controller_state.left_stick_x * max_speed
        vel_y = controller_state.left_stick_y * max_speed

        wheel_speed = vector_to_wheel_speed(vel_x, vel_y, 0, wheel_radius, bot_radius)
        wheel_rpms = []
        for speed in wheel_speed:
            wheel_rpms.append((60 * speed) / (3.14 * 2 * wheel_radius))
            
        back_wheel_rpm = wheel_rpms[0]
        left_wheel_rpm = wheel_rpms[1]
        right_wheel_rpm = wheel_rpms[2]

        print(f"RPM_BACK: {back_wheel_rpm}")
        print(f"RPM_LEFT: {left_wheel_rpm}")
        print(f"RPM_RIGHT: {right_wheel_rpm}\n")

        frame = gen_wheel_rpm_command(MODE_MASK_RPM.MANUAL, MOTOR_ID_MASK.MOTOR_BACK, back_wheel_rpm)
        can_pub.publish(frame)

        frame = gen_wheel_rpm_command(MODE_MASK_RPM.MANUAL, MOTOR_ID_MASK.MOTOR_LEFT, left_wheel_rpm)
        can_pub.publish(frame)

        frame = gen_wheel_rpm_command(MODE_MASK_RPM.MANUAL, MOTOR_ID_MASK.MOTOR_RIGHT, right_wheel_rpm)
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

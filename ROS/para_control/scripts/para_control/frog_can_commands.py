""" frog_can_commands.py 

Generate CAN Frames for controlling the F.R.O.G Robot 
"""

import struct

from can_msgs.msg import Frame
from enum import IntEnum, unique


@unique
class BOT_STATE(IntEnum):
    BOOT        = 0
    STANDBY     = 1
    AUTONOMOUS  = 2
    MANUAL      = 3


@unique
class MODE_MASK_RPM(IntEnum):
    AUTONOMOUS      = 0x0040
    MANUAL          = 0x0020


@unique
class MODE_MASK_DUTY(IntEnum):
    AUTONOMOUS     = 0x0400
    MANUAL         = 0x0200


@unique
class MODE_MASK_CURRENT(IntEnum):
    AUTONOMOUS  = 0x0100
    MANUAL      = 0x0080


@unique
class MOTOR_ID_MASK(IntEnum):
    MOTOR_RIGHT     = 0x0001
    MOTOR_BACK     = 0x0002
    MOTOR_LEFT     = 0x0003


def state_to_id(state: BOT_STATE):
    """ Take a bot state and return the associated CAN ID that must be sent """
    if state == BOT_STATE.BOOT: 
        return False
    elif state == BOT_STATE.STANDBY:
        return 0x0011
    elif state == BOT_STATE.AUTONOMOUS:
        return 0x0012
    elif state == BOT_STATE.MANUAL:
        return 0x0013
    else:
        return False


def gen_state_command(state: BOT_STATE) -> Frame:
    """ Generate a state command CAN frame """
    id = state_to_id(state)
    frame = Frame()
    frame.id = id
    frame.dlc = 0
    frame.data = [0, 0, 0, 0, 0, 0, 0, 0]
    return frame


def gen_heartbeat_command() -> Frame:
    frame = Frame()
    frame.id = 0x0001
    frame.dlc = 0
    frame.data = [0, 0, 0, 0, 0, 0, 0, 0]
    return frame


def gen_wheel_rpm_command(mode: MODE_MASK_RPM, motor_id: MOTOR_ID_MASK, rpm: float) -> Frame:
    frame = Frame()
    frame.id = motor_id | mode

    # Float, little endian, E-RPM
    rpm_bytes = bytearray(struct.pack("<f", rpm * 7))
    frame.data = rpm_bytes
    frame.dlc = len(rpm_bytes)
    return frame


def gen_wheel_current_command(mode: MODE_MASK_CURRENT, motor_id: MOTOR_ID_MASK, current: float) -> Frame:
    frame = Frame()
    frame.id = motor_id | mode
    
    # Float, little endian
    current_bytes = bytearray(struct.pack("<f", current))
    frame.data = current_bytes
    frame.dlc = len(current_bytes)
    return frame


def gen_wheel_duty_command(mode: MODE_MASK_DUTY, motor_id: MOTOR_ID_MASK, duty_cycle: float) -> Frame:
    frame = Frame()
    frame.id = motor_id | mode

    # Float, little endian
    duty_cycle_bytes = bytearray(struct.pack("<f", duty_cycle))
    frame.data = duty_cycle_bytes
    frame.dlc = len(duty_cycle_bytes)
    return frame


if __name__ == "__main__":
    frame = gen_wheel_rpm_command(MODE_MASK_RPM.MANUAL_RPM, MOTOR_ID_MASK.MOTOR_RIGHT, 1000)
    print(hex(frame.id))
    print(frame.data)
    
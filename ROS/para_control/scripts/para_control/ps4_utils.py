""" ps4_utils.py

Utilities for using ps4 controller with ROS
"""

from enum import IntEnum, unique
from sensor_msgs.msg import Joy


@unique
class PS4ControllerAxisIndices(IntEnum):
    """ PS4 Controller `ROS Joy` message axis array indices
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
class PS4ControllerButtonIndices(IntEnum):
    """ PS4 Controller 'ROS Joy' message button array indices
    """
    X                   = 0
    CIRCLE              = 1
    TRIANGLE            = 2
    SQUARE              = 3
    LB                  = 4
    RB                  = 5
    SHARE               = 8
    OPTIONS             = 9
    POWER               = 10
    LEFT_STICK_CLICK    = 11
    RIGHT_STICK_CLICK   = 12


class PS4ControllerState():
    """ Class representing the state of a PS4 controller """
    def __init__(self):
        # Axes 
        self.dpad_x = 0
        self.dpad_y = 0

        self.left_stick_x = 0
        self.left_stick_y = 0

        self.right_stick_x = 0
        self.right_stick_y = 0

        self.left_trigger = 0
        self.right_trigger = 0

        # Buttons
        self.x = 0
        self.circle = 0
        self.square = 0
        self.triangle = 0
        
        self.share = 0
        self.options = 0
        self.power = 0

        self.left_bumper = 0
        self.right_bumper = 0

        self.left_stick_click = 0
        self.right_stick_click = 0

    def update_state_from_joy(self, state: Joy):
        """ Update controller state from a Joy message """
        # Axis (sticks, dpad, trigger)
        self.dpad_x = state.axes[PS4ControllerAxisIndices.DPAD_X]
        self.dpad_y = state.axes[PS4ControllerAxisIndices.DPAD_Y]

        self.left_stick_x = state.axes[PS4ControllerAxisIndices.LEFT_X]
        self.left_stick_y = state.axes[PS4ControllerAxisIndices.LEFT_Y]

        self.right_stick_x = state.axes[PS4ControllerAxisIndices.RIGHT_X] 
        self.right_stick_y = state.axes[PS4ControllerAxisIndices.RIGHT_Y]

        self.left_trigger = state.axes[PS4ControllerAxisIndices.LEFT_TRIGGER]
        self.right_trigger = state.axes[PS4ControllerAxisIndices.RIGHT_TRIGGER]

        # Buttons
        self.x = state.buttons[PS4ControllerButtonIndices.X]
        self.circle = state.buttons[PS4ControllerButtonIndices.CIRCLE]
        self.square = state.buttons[PS4ControllerButtonIndices.SQUARE]
        self.triangle = state.buttons[PS4ControllerButtonIndices.TRIANGLE]

        self.share = state.buttons[PS4ControllerButtonIndices.SHARE]
        self.options = state.buttons[PS4ControllerButtonIndices.OPTIONS]
        self.power = state.buttons[PS4ControllerButtonIndices.POWER]

        self.left_bumper = state.buttons[PS4ControllerButtonIndices.LB]
        self.right_bumper = state.buttons[PS4ControllerButtonIndices.RB]

        self.left_stick_click = state.buttons[PS4ControllerButtonIndices.LEFT_STICK_CLICK]
        self.right_stick_click = state.buttons[PS4ControllerButtonIndices.RIGHT_STICK_CLICK]

    def __str__(self):
        """ Return string representation of the controller state """
        ret_str = f"Left Stick: ({self.left_stick_x}, {self.left_stick_y})\n" \
                  f"Right Stick: ({self.right_stick_x}, {self.right_stick_y})\n" \
                  f"D-Pad: ({self.dpad_x}, {self.dpad_y})\n" \
                  f"Left Trigger: {self.left_trigger}\n" \
                  f"Right Trigger: {self.right_trigger}\n" \
                  f"X: {self.x}\n" \
                  f"Circle: {self.circle}\n" \
                  f"Triangle: {self.triangle}\n" \
                  f"Square: {self.square}\n" \
                  f"Options: {self.options}\n" \
                  f"Share: {self.share}\n" \
                  f"Power: {self.power}\n" \
                  f"Left Bumper: {self.left_bumper}\n" \
                  f"Right Bumber: {self.right_bumper}\n" \
                  f"Left Stick Click: {self.left_stick_click}\n" \
                  f"Right Stick Click: {self.right_stick_click}\n" \
                
        return  ret_str

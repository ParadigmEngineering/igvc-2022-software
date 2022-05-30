""" xbox_utils.py

Utilities for converting Joy messages to a 
an easier to use format. 
"""

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


class XboxControllerState():
    """ XboxControllerState
    
    Represents the inputs of an xbox 360 controller. 

    Buttons (0, 1): Pressed = 1, Released = 0 
    Trigger (float): Squeezed Fully = -1, Released = 1 
    Joypad (float): 
                +y 1
                |
                |
    +x (1) -----O----- -1
                |
                |
                -1
    """

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
        self.a = 0
        self.b = 0
        self.x = 0
        self.y = 0

        self.back = 0
        self.start = 0
        self.power = 0

        self.left_bumper = 0
        self.right_bumper = 0

        self.left_stick_click = 0
        self.right_stick_click = 0
    
    def update_state_from_joy(self, state: Joy):
        """ Update controller state from a Joy message """
        self.dpad_x = state.axes[ControllerAxisIndices.DPAD_X]
        self.dpad_y = state.axes[ControllerAxisIndices.DPAD_Y]

        self.left_stick_x = state.axes[ControllerAxisIndices.LEFT_X]
        self.left_stick_y = state.axes[ControllerAxisIndices.LEFT_Y]

        self.right_stick_x = state.axes[ControllerAxisIndices.RIGHT_X] 
        self.right_stick_y = state.axes[ControllerAxisIndices.RIGHT_Y]

        self.left_trigger = state.axes[ControllerAxisIndices.LEFT_TRIGGER]
        self.right_trigger = state.axes[ControllerAxisIndices.RIGHT_TRIGGER]

        # Buttons
        self.a = state.buttons[ControllerButtonIndices.A]
        self.b = state.buttons[ControllerButtonIndices.B]
        self.x = state.buttons[ControllerButtonIndices.X]
        self.y = state.buttons[ControllerButtonIndices.Y]

        self.back = state.buttons[ControllerButtonIndices.BACK]
        self.start = state.buttons[ControllerButtonIndices.START]
        self.power = state.buttons[ControllerButtonIndices.POWER]

        self.left_bumper = state.buttons[ControllerButtonIndices.LB]
        self.right_bumper = state.buttons[ControllerButtonIndices.RB]

        self.left_stick_click = state.buttons[ControllerButtonIndices.LEFT_STICK_CLICK]
        self.right_stick_click = state.buttons[ControllerButtonIndices.RIGHT_STICK_CLICK]

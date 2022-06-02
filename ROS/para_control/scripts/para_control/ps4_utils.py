""" ps4_utils.py

Utilities for using ps4 controller with ROS
"""

from para_control.xbox_utils import ControllerAxisIndices, ControllerButtonIndices
from sensor_msgs.msg import Joy


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
        self.square = 0
        self.triangle = 0
        self.circle = 0

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
            self.x = state.buttons[ControllerButtonIndices.A]
            self.circle = state.buttons[ControllerButtonIndices.B]
            self.square = state.buttons[ControllerButtonIndices.X]
            self.triangle = state.buttons[ControllerButtonIndices.Y]

            self.back = state.buttons[ControllerButtonIndices.BACK]
            self.start = state.buttons[ControllerButtonIndices.START]
            self.power = state.buttons[ControllerButtonIndices.POWER]

            self.left_bumper = state.buttons[ControllerButtonIndices.LB]
            self.right_bumper = state.buttons[ControllerButtonIndices.RB]

            self.left_stick_click = state.buttons[ControllerButtonIndices.LEFT_STICK_CLICK]
            self.right_stick_click = state.buttons[ControllerButtonIndices.RIGHT_STICK_CLICK]

    def __str__(self):
        """ Return string representation of the controller state """
        ret_str = f"Left Stick: ({self.left_stick_x}, {self.left_stick_y}\n" \
                  f"Right Stick: ({self.right_stick_x}. {self.right_stick_y})\n" \
                  f"D-Pad: ({self.dpad_x}, {self.dpad_y})" \
                  f"Left Trigger: {self.left_trigger}" \
                  f"Right Trigger: {self.right_trigger}" \
                  f"X: {self.x}" \
                  f"Circle: {self.circle}" \
                  f"Square: {self.square}" \
                  f"Triangle: {self.triangle}" \
                  f"Back: {self.back}" \
                  f"Start: {self.start}" \
                  f"Power: {self.power}" \
                  f"Left Bumper: {self.left_bumper}" \
                  f"Right Bumber: {self.right_bumper}" \
                  f"Left Stick Click: {self.left_stick_click}" \
                  f"Right Stick Click: {self.right_stick_click}" \
                
        return  ret_str

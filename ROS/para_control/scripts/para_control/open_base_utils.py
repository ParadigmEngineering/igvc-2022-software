""" open_base_utils.py

Utilities for interacting witrh open_base message types. 
"""

from enum import IntEnum, unique 


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

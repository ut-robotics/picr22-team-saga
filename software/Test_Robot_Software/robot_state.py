from enum import Enum

class State(Enum):
    MANUAL = 0
    INITIAL = 1
    FIND_A_BALL = 2
    DRIVE_TO_OTHER_HALF = 3
    FOLLOW_A_BALL = 4
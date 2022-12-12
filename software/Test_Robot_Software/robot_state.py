import enum
import numpy as np


class State(enum.Enum):
    MANUAL = 0
    INITIAL = 1
    FIND_A_BALL = 2
    FOLLOW_A_BALL = 3   


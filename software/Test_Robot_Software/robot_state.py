import enum
import numpy as np

class State(enum.Enum):
    INITIAL = 0
    FIND_A_BALL = 1
    FOLLOW_A_BALL = 2


class StateMachine:
    def __init__(self):
        self.current_state = State.INITIAL
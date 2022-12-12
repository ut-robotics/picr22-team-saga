from robot_state import State
from api import RobotMovement

class StateMachine:
    """
    This class is responsible for switching the states
    of the robot as implementing the state machine logic
    """

    def __init__(self, robot_movement):
        self.robot_movement = robot_movement
        self.state = State.FIND_A_BALL
        self.counter = 0

    def current_state(self, ball_x, ball_y):

        if self.state == State.FIND_A_BALL:
            self.find_a_ball(ball_x)
        
        elif self.state == State.FOLLOW_A_BALL:
            self.follow_a_ball(ball_x, ball_y)
        
    def find_a_ball(self, ball_x):
        pass

    def follow_a_ball(self, ball_x, ball_y):
        pass
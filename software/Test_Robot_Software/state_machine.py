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

    def run_current_state(self, ball_x, ball_y):

        if self.state == State.FIND_A_BALL:
            self.find_a_ball(ball_x)
        
        elif self.state == State.FOLLOW_A_BALL:
            self.follow_a_ball(ball_x, ball_y)
        
    def find_a_ball(self, ball_x):
        if ball_x is None:
            self.robot_movement.move(0, 0, -8)
        else:
            self.state = State.FOLLOW_A_BALL
            self.robot_movement.move(0, 0, 0)

    def follow_a_ball(self, ball_x, ball_y):
        
        if ball_x is None:
            self.state = State.FIND_A_BALL
        else:
        
            max_speed_x = 10
            max_speed_y = 30
            
            speed_x = ball_x * 18
            if (speed_x > max_speed_x):
                speed_x = max_speed_x
            if (speed_x < -max_speed_x):
                speed_x = -max_speed_x
            
            if (ball_y < -0.4):
                ball_y = 0.0
            speed_y = (ball_y + 0.4) * 60
            if (speed_y > max_speed_y):
                speed_y = max_speed_y
            
            rot_speed = -speed_x * 1.5
            
            self.robot_movement.move(speed_x, speed_y, rot_speed)
        
if __name__ == "__main__":
    state_machine = StateMachine()
    state_machine.robot_movement.move(0, 0, 0)

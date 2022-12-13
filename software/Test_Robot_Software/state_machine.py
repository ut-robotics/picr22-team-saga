from robot_state import State
from api import RobotMovement

import cv2

class StateMachine:
    """
    This class is responsible for switching the states
    of the robot as implementing the state machine logic
    """

    def __init__(self, robot_movement):
        self.robot_movement = robot_movement
        self.state = State.FIND_A_BALL
        
        self.max_speed_x = 10
        self.max_speed_y = 40

    # def run_current_state(self, ball_x, ball_y):
        
    #     if self.state == State.FIND_A_BALL:
    #         self.find_a_ball(ball_x)
        
    #     elif self.state == State.FOLLOW_A_BALL:
    #         self.follow_a_ball(ball_x, ball_y)

    def run_current_state(self, processed_data, frame_width, frame_height):

        if processed_data.balls:
            print("ball exists")
            largest = processed_data.balls[-1] 
            cv2.circle(processed_data.debug_frame, (largest.x, largest.y), 20, (255, 0, 255), -1)                

            ball_x = (largest.x - frame_width/2) / (frame_width/2) 
            ball_y = (frame_height/2 - largest.y) / (frame_height/2)
            print(ball_x, ball_y)

            if self.state == State.FIND_A_BALL:
                self.find_a_ball(ball_x=ball_x)
            elif self.state == State.FOLLOW_A_BALL:
                self.follow_a_ball(ball_x=ball_x, ball_y=ball_y)
        
        else:
            self.robot_movement.move(0, 0, -8)
            
    def find_a_ball(self, ball_x):
        
        self.state = State.FOLLOW_A_BALL
        self.robot_movement.move(0, 0, 0)

    def follow_a_ball(self, ball_x, ball_y):

        speed_x = (ball_x - 0.225) * 16.0

        if (speed_x > self.max_speed_x):
            speed_x = self.max_speed_x
        if (speed_x < -self.max_speed_x):
            speed_x = -self.max_speed_x
        
        if (ball_y < -0.4):
            speed_y = 0.0
            
        else:                    
            speed_y = (ball_y + 0.4)  * 60.0 
        
        if (speed_y > self.max_speed_y):
            speed_y = self.max_speed_y
        else:
            pass
        
        rot_speed = -speed_x * 1.25
        
        self.robot_movement.move(speed_x, speed_y, rot_speed)  
    
        
if __name__ == "__main__":
    state_machine = StateMachine()
    state_machine.robot_movement.move(0, 0, 0)

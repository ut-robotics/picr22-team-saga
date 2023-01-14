from robot_state import State
from api import RobotMovement

import cv2
from time import time, sleep
import numpy as np

class StateMachine:
    """
    This class is responsible for switching the states
    of the robot as implementing the state machine logic
    """

    def __init__(self, robot_movement, frame_width, frame_height):
        self.robot_movement = robot_movement
        self.state = State.FIND_A_BALL
        
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        self.max_speed_x = 10
        self.max_speed_y = 60

        self.counter = 0

        # self.basket_exists = False
        self.basket = None
        self.basket_m = []
        self.basket_b = []

    # def run_current_state(self, ball_x, ball_y):
        
    #     if self.state == State.FIND_A_BALL:
    #         self.find_a_ball(ball_x)
        
    #     elif self.state == State.FOLLOW_A_BALL:
    #         self.follow_a_ball(ball_x, ball_y)

    def run_current_state(self, processed_data):
        print(self.state)
        
        if processed_data.balls:
            self.counter = 0
            print("ball exists")
            largest = processed_data.balls[-1] 
            cv2.circle(processed_data.debug_frame, (largest.x, largest.y), 20, (255, 0, 255), -1)                

            ball_x = (largest.x - self.frame_width/2) / (self.frame_width/2) 
            ball_y = (self.frame_height/2 - largest.y) / (self.frame_height/2)
            print(ball_x, ball_y)

            if self.state == State.FIND_A_BALL:
                self.find_a_ball(ball_x=ball_x)
            elif self.state == State.FOLLOW_A_BALL:
                self.follow_a_ball(ball_x=ball_x, ball_y=ball_y)
        
        else:            
            # self.robot_movement.move(0, 0, -7)
            self.find_a_ball(ball_x=None)
            self.counter += 1
            if self.counter > 90:
                self.state = State.DRIVE_TO_OTHER_HALF                
                # print(self.state)
        # if processed_data.basket_m:
        #     self.basket_m.append(processed_data.basket_m.size)
        #     self.basket_m_size = np.array(self.basket_m[:-1][:10])
        
        # if processed_data.basket_b:
        #     self.basket_b.append(processed_data.basket_b.size)
        #     self.basket_b_size = np.array(self.basket_b[:-1][:10])
        
        if self.state == State.DRIVE_TO_OTHER_HALF:
            self.drive_to_other_half(processed_data=processed_data)
        
        # self.basket = processed_data.basket_m


    def find_a_ball(self, ball_x):
        # print(self.state)
        if (ball_x is None):     
            self.robot_movement.move(0, 0, -5)
            # pass     
            
        else:
            self.counter = 0
            self.state = State.FOLLOW_A_BALL
            # self.robot_movement.move(0, 0, 0)

    def drive_to_other_half(self, processed_data):
        if (processed_data.balls):
            # self.counter = 0
            self.state = State.FIND_A_BALL
        
        else:
            
            # if (processed_data.basket_m.size < 3000):
            #     self.basket = processed_data.basket_m
            # elif (processed_data.basket_b.size < 3000):
            #     self.basket= processed_data.basket_b
            # else:
            #     self.basket = None
            # if (np.mean(self.basket_b_size) > np.mean(self.basket_m_size)):
            #     self.basket = processed_data.basket_m 
            #     print("Farther basket: Magenta")
            #     # print(self.basket.size)
            #     print(self.basket_m_size)
            # else:
            #     self.basket = processed_data.basket_b
            #     print("Farther basket: Blue")
            #     # print(self.basket.size)
            #     print(self.basket_b_size)
            # self.robot_movement.move(0, 0, 0)

            # if (processed_data.basket_m.size < 2000):
            if (processed_data.basket_m.size == -1):
                # self.basket = processed_data.basket_m
                self.basket = processed_data.basket_b
                # print(processed_data.basket_m)
                print("Farther_basket: Blue")
            else:
                # self.basket = processed_data.basket_b
                self.basket = processed_data.basket_m
                # print(processed_data.basket_b)
                print("Farther_basket: Magenta")
            
            # if(self.basket.size < 900.0):
            if(self.basket.exists):
                
                self.basket_x = self.basket.x
                self.basket_y = self.basket.y

                target_x = (self.basket_x - self.frame_width/2) / (self.frame_width/2) 
                target_y = (self.frame_height/2 - self.basket_y) / (self.frame_height/2)

                target_speed_x = target_x * 12
                if (target_speed_x > self.max_speed_x):
                    target_speed_x = self.max_speed_x
                if (target_speed_x < -self.max_speed_x):
                    target_speed_x = -self.max_speed_x
                
                target_speed_y = target_y * 80
                if (target_speed_y > self.max_speed_y):
                    target_speed_y = self.max_speed_y
                else:
                    pass

                target_rot_speed = -target_speed_x

                self.robot_movement.move(target_speed_x, target_speed_y, target_rot_speed)
            
            else:
                self.robot_movement.move(0, 0, -6)

            
    def follow_a_ball(self, ball_x, ball_y):
        
        if (ball_x is None):
            self.state = State.FIND_A_BALL
        
        else:
            self.counter = 0

            speed_x = (ball_x - 0.225) * 16.0
            # speed_x = ball_x * 16.0

            if (speed_x > self.max_speed_x):
                speed_x = self.max_speed_x
            if (speed_x < -self.max_speed_x):
                speed_x = -self.max_speed_x
            
            if (ball_y < -0.5):
                speed_y = 0.0
            # if (ball_y < -0.0):
            #     speed_y = 0.0
                
            else:                    
                speed_y = (ball_y + 0.5)  * 80.0 
                # speed_y = ball_y   * 60.0 
            
            if (speed_y > self.max_speed_y):
                speed_y = self.max_speed_y
            else:
                pass
            
            rot_speed = -speed_x * 1.25
            
            self.robot_movement.move(speed_x, speed_y, rot_speed)  

        
if __name__ == "__main__":
    state_machine = StateMachine()
    # state_machine.robot_movement.move(0, 0, 0)

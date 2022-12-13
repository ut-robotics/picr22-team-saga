from camera import RealsenseCamera
from image_processor import ImageProcessor
from api import RobotMovement
from robot_state import State
from state_machine import StateMachine
from time import time

import cv2

class Main:
    """
    This class combines all tasks:
    finding a ball, following a ball, etc
    """

    def __init__(self, robot_movement):

        self.debug = True
        self.cam = RealsenseCamera(exposure=100)
        self.processor = ImageProcessor(self.cam, debug=self.debug)
        self.processor.start()    
        
        self.FRAME_WIDTH = self.cam.rgb_width
        self.FRAME_HEIGHT = self.cam.rgb_height

        self.robot_movement = RobotMovement()
        self.state_machine = StateMachine(self.robot_movement, self.FRAME_WIDTH, self.FRAME_HEIGHT )
        
        
        

if __name__ == "__main__":
    main = Main(robot_movement=RobotMovement())
# 
    start = time()
    fps = 0
    frame = 0
    frame_cnt = 0

    try:
        while True:
            processed_data = main.processor.process_frame(aligned_depth=False) 
            if processed_data:
                main.current_state = main.state_machine.run_current_state(processed_data=processed_data)          
            
            # if processed_data.balls:
            #     largest = processed_data.balls[-1] 
            #     cv2.circle(processed_data.debug_frame, (largest.x, largest.y), 20, (255, 0, 255), -1)                

            #     ball_x = (largest.x - main.cam.rgb_width/2) / (main.cam.rgb_width/2) 
            #     ball_y = (main.cam.rgb_height/2 - largest.y) / (main.cam.rgb_height/2)
            #     print(ball_x, ball_y)
            
            #     main.current_state = main.state_machine.run_current_state(ball_x=ball_x, ball_y=ball_y)
                
            # else:
            #     # main.robot_movement.move(0, 0, -8)
            #     main.state_machine.turn(-8)

            # print(largest)

            frame_cnt +=1
            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processed_data.balls)))

            if main.debug:
                debug_frame = processed_data.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break
    
    except KeyboardInterrupt:
        print("Keybaord Interrupt ..")
        print("Closing ....")

    finally:
        main.processor.stop()
        cv2.destroyAllWindows()
                

        
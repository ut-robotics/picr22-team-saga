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

        self.robot_movement = RobotMovement()
        self.state_machine = StateMachine(self.robot_movement)
        self.current_state = State.FIND_A_BALL

if __name__ == "__main__":
    main = Main()

    start = time()
    fps = 0
    frame = 0
    frame_cnt = 0

    try:
        while True:
            processed_data = main.processor.process_frame(aligned_depth=False)
            largest = processed_data.balls[-1]

            if largest:
                cv2.circle(processed_data.debug_frame, (largest.x, largest.y), 20, (255, 0, 255), -1)
            
            frame_cnt +=1

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processed_data.balls)))

                #if (frame_cnt > 1000):
                #    break

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
                

        
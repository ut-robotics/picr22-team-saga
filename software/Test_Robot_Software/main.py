from camera import RealsenseCamera
from image_processor import ImageProcessor
from api import RobotMovement
from robot_state import State

import cv2

class Main:
    """
    This class combines all tasks:
    finding a ball, following a ball, etc
    """

    def __init__(self):
        self.cam = RealsenseCamera()
        self.cam.open()
        self.robot_movement = RobotMovement()
        
        self.current_state = State.INITIAL
        
from api import RobotMovement
from robot_state import State

class MainboardComm:
    """
    This class is there just for veryfying everythin works
    whena speed command is sent to the MCU
    via serial
    """

    def __init__(self):
        self.current_state = State.MANUAL
    

if __name__ == "__main__":
    mainboard_comm = MainboardComm()
    robot = RobotMovement()
    while True:
        try:
            if mainboard_comm.current_state == State.MANUAL:
                robot.move(0, 15, 0)

        except KeyboardInterrupt:
            robot.move(0, 0, 0)
            print("Keyboard Interrupt ...")
            print("Closing ...")
            robot.close()
            break

        # finally:
        #     # robot.close()
        #     pass
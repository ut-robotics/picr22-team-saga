import api
import time
from robot_state import State

# def main():
#     try:
#         while True:
#             if (robot_state.StateMachine.current_state == robot_state.State.MANUAL):            
#                 api.RobotMovement.move(0, 12, 0)       
#                 print("Moving ..")

#     except KeyboardInterrupt:
#         print("Keyboard Interrupt ..")
#         api.RobotMovement.move(0.5, 0.5, 0.5)
#         print("Keyboard Interrupt ...")
#         print("Closing ...")
    
#     finally:
#         api.RobotMovement.close()

# if __name__ == "__main__":
#     main()

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
    while True:
        try:
            if mainboard_comm.current_state == State.MANUAL:
                api.RobotMovement.move(0, 12, 0)

        except KeyboardInterrupt:
            api.RobotMovement.move(0, 0, 0)
            print("Keyboard Interrupt ...")
            print("Closing ...")
            break

        finally:
            api.RobotMovement.close()
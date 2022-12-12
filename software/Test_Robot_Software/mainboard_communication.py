import api
import time
import robot_state

def main():
    try:
        while True:
            if (robot_state.StateMachine.current_state == robot_state.State.MANUAL):            
                api.RobotMovement.move(0, 12, 0)       
                print("Moving ..")

    except KeyboardInterrupt:
        print("Keyboard Interrupt ..")
        api.RobotMovement.move(0.5, 0.5, 0.5)
        print("Keyboard Interrupt ...")
        print("Closing ...")
    
    finally:
        api.RobotMovement.close()

if __name__ == "__main__":
    main()
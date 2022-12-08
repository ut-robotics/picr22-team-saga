import api
import time

def main():
    try:
        while True:
            api.move(0, 12, 0)       
            print("Moving ..")

    except KeyboardInterrupt:
        print("Keyboard Interrupt ..")
        api.move(0.5, 0.5, 0.5)
        print("Keyboard Interrupt ...")
        print("Clsoing ...")
    
    finally:
        api.close()

if __name__ == "__main__":
    main()
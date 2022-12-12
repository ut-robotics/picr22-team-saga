import math

import sys
import time
import struct

import serial
# s = serial.Serial("/dev/ttyACM0", baudrate=115200)

STM32_HWID = "USB VID:PID=0483:5740"
wHEEL_ANGLES = [240, 120, 0]

class RobotMovement:

    def __init__(self):
        ports = serial.tools.ports.comports()
        devices = {}

        for port, _, hwid in sorted(ports):
            devices[hwid] = port
        
        for hwid in devices.keys():
            if STM32_HWID in hwid:
                serial_port = devices[hwid]
                break
        
        if serial_port is None:
            print("Serial port not found")
        
        self.ser = serial.Serial(serial_port, 115200)

    def sendThrower(self, thrower_speed):    
        pass

    def sendMotorSpeed(self, motors):
        print(motors)
        data = struct.pack('<hhhHBH', motors[0], motors[2], motors[1], 48, 0, 0xAAAA)
        self.ser.write(data)
        self.ser.read()    
        
    def close(self):
        self.ser.close() 


    def move(self, x_speed, y_speed, rot_speed):
        
        # disable_failsafe = 1
        speeds = [0, 0, 0]
        robotSpeed = math.sqrt(x_speed * x_speed + y_speed * y_speed)
        robotDirectionAngle = math.atan2(y_speed, x_speed)

        for wheelAngle, i in zip(wHEEL_ANGLES, range(3)):
            wheelLinearVelocity = robotSpeed * math.cos(robotDirectionAngle - wheelAngle / 180.0 * math.pi) + rot_speed
            speeds[i] = wheelLinearVelocity

        print(x_speed, y_speed, rot_speed)
        print(list(map(lambda x: int(x), speeds)))
        RobotMovement.sendMotorSpeed(list(map(lambda x: int(x), speeds)))    


if __name__ == "__main__":
    # Read the response, an array of byte, .tobytes() gives us a bytearray.
    # speed = int(sys.argv[1])
    # sendThrower(speed)

    # if sys.argv[2] == "M":
    #     motors = list(map(int, sys.argv[3:]))
    #     sendMotorSpeed(motors)
    #     print(motors)
    # else:
    #     move(int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]))


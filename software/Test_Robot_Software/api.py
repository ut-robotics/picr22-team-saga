import math

#import usb.core
#import usb.util
import sys
import time
import struct
import serial.tools.list_ports

# s = serial.Serial("/dev/ttyACM0", baudrate=115200)
# find our device
#dev = usb.core.find(idVendor=0x0483, idProduct=0x5750)

# was it found?
# if dev is None:
#     raise ValueError('Device not found')

# i = dev[0].interfaces()[0].bInterfaceNumber
# if dev.is_kernel_driver_active(i):
#     try:
#         dev.detach_kernel_driver(i)
#     except usb.core.USBError as e:
#         sys.exit("Could not detatch kernel driver from interface({0}): {1}".format(i, str(e)))

# # set the active configuration. With no arguments, the first
# # configuration will be the active one
# dev.set_configuration()

# # get an endpoint instance
# cfg = dev.get_active_configuration()
# intf = cfg[(0, 0)]

# ep = usb.util.find_descriptor(
#     intf,
#     # match the first OUT endpoint
#     custom_match= \
#         lambda e: \
#             usb.util.endpoint_direction(e.bEndpointAddress) == \
#             usb.util.ENDPOINT_OUT)

# # print("print ep")
# # print(ep)
# # print("print cfg")
# # print(cfg)
# # print("print intf")
# # print(intf)
# # print("report")

# endpoint_in = dev[0][(0, 0)][0]
# endpoint_out = dev[0][(0, 0)][1]


# def sendThrower(thrower_speed):
#     # for i in range(2):
#     #     endpoint_out.write(bytes([1, thrower_speed >> 8, thrower_speed & 0xFF, 0]))
#     pass

# def sendMotorSpeed(motors):
#     print(motors)
#     data = struct.pack('<hhhHBH', motors[0], motors[2], motors[1], 48, 0, 0xAAAA)
#     s.write(data)
#     s.read()

#     # endpoint_out.write(data)
#     # for i in range(2):
#     #     endpoint_out.write(bytes([2, motors[2] & 0xFF, motors[1] & 0xFF, motors[0] & 0xFF]))
#         # endpoint_out.write(bytes([2, motors[0] & 0xFF, motors[1] & 0xFF, motors[2] & 0xFF]))
    
# def close():
#     s.close() 


# def move(x_speed, y_speed, rot_speed):
#     # time.sleep(0.5)
#     disable_failsafe = 1
#     speeds = [0, 0, 0]
#     robotSpeed = math.sqrt(x_speed * x_speed + y_speed * y_speed)
#     robotDirectionAngle = math.atan2(y_speed, x_speed)

#     for wheelAngle, i in zip([240, 120, 0], range(3)):
#         wheelLinearVelocity = robotSpeed * math.cos(robotDirectionAngle - wheelAngle / 180.0 * math.pi) + rot_speed
#         speeds[i] = wheelLinearVelocity

#     print(x_speed, y_speed, rot_speed)
#     print(list(map(lambda x: int(x), speeds)))
#     sendMotorSpeed(list(map(lambda x: int(x), speeds)))
#     # sendMotorSpeed([-50, -60, -70] )
   


# if __name__ == "__main__":
#     # Read the response, an array of byte, .tobytes() gives us a bytearray.
#     speed = int(sys.argv[1])
#     sendThrower(speed)

#     if sys.argv[2] == "M":
#         motors = list(map(int, sys.argv[3:]))
#         sendMotorSpeed(motors)
#         print(motors)
#     else:
#         move(int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]))

# while(True):

#     # Send a command to the Teensy
#     #endpoint_out.write( bytes([1, 0, 0xA0, 1]) )
#     try:
#         buffer = dev.read(endpoint_in.bEndpointAddress, 4, 1000).tobytes()
#     except:
#         print("time out")
#         exit(0)
#     # Decode and print the zero terminated string response
#     #n = buffer.index(0)
#     print( buffer)

# assert ep is not None

STM32_HWID = "USB VID:PID=0483:5740"
wHEEL_ANGLES = [240, 120, 0]

class RobotMovement:

    def __init__(self):
        ports = serial.tools.list_ports.comports()
        devices = {}

        for port, _, hwid in sorted(ports):
            devices[hwid] = port
        
        for hwid in devices.keys():
            if STM32_HWID in hwid:
                serial_port = devices[hwid]
                break
        
        if serial_port is None:
            raise ValueError("Could not find the COM port")
            # raise Exception("Could not find the COM port")
        
        self.ser = serial.Serial(serial_port, 115200)
        # self.ser.open()

    def sendThrower(self, thrower_speed):    
        pass

    def sendMotorSpeed(self, motors):
        print(motors)
        data = struct.pack('<hhhHBH', motors[0], motors[2], motors[1], 48, 0, 0xAAAA)
        self.ser.write(data)
        # self.ser.read()    

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
        self.sendMotorSpeed(list(map(lambda x: int(x), speeds)))    
    
    def close(self):
        self.ser.close()

if __name__ == "__main__":
    robot = RobotMovement()
    robot.move(0, 0, 0)


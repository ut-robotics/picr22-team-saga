import pyrealsense2 as rs
import camera as Camera
import socket
import sys
import pickle
import struct ### new code

def main():
    try:
        clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        print(clientsocket)
        clientsocket.connect(('192.168.3.62',8089))
        print(clientsocket)
        camera = Camera.RealsenseCamera()
        camera.open()
        while True:
            frame = camera.get_color_frame()
            data = pickle.dumps(frame) ### new code
            print(len(data), frame.shape[0], frame.shape[1])
            packed = struct.pack("I", len(data))+data
            #print(packed)
            clientsocket.sendall(packed) ### new code
    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()
import socket
import sys
import cv2
import pickle
import numpy as np
import struct ## new
import time
import threading

from tkinter import *
from PIL import Image, ImageTk

HOST=''
PORT=8089

s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print('Socket created')

s.bind((HOST,PORT))
print('Socket bind complete')
s.listen(10)
print('Socket now listening')

conn,addr=s.accept()





class Viewer:
    def __init__(self):
        self.thread = threading.Thread(target=self.videoF, args=())
        self.thread.start()
        self.panel = None



    def videoF(self):
        ### new
        data = b""
        payload_size = struct.calcsize("I") 
        while True:
            while len(data) < payload_size:
                data += conn.recv(4096)
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("I", packed_msg_size)[0]
            print(msg_size)
            while len(data) < msg_size:
                data += conn.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame=pickle.loads(frame_data)
            print(frame.shape[0], frame.shape[1])
            
            #cv2.imwrite("test.jpg", frame)
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            im = Image.fromarray(image)
            imgtk = ImageTk.PhotoImage(image=im)
            #lable.image = imgtk
            #cv2.imwrite("test1.jpg", image)
            # show the image, provide window name first
            #cv2.imshow("im", image)
            ###

    # if the panel is not None, we need to initialize it
            if self.panel is None:
                self.panel = Label(image=imgtk)
                self.panel.image = imgtk
                self.panel.pack(side="left", padx=10, pady=10)
            
                # otherwise, simply update the panel
            else:
                self.panel.configure(image=imgtk)
                self.panel.image = imgtk

win = Tk()
win.geometry("480x848")
v = Viewer()

win.mainloop()
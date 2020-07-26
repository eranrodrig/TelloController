import threading
from easytello import tello
import cv2
from Constants import *
import numpy as np


class RealTelloController:

    def __init__(self):
        self.frame = np.zeros([64, 64], np.uint8)
        self.drone = tello.Tello(debug=False)
        self.video_thread = threading.Thread(target=self._start_stream, daemon=True)
        self.video_thread.start()

    def take_picture(self):
        return self.frame

    def cw(self):
        self.drone.cw(rotation_epsilon * 180)

    def ccw(self):
        self.drone.ccw(rotation_epsilon * 180)

    def left(self):
        self.drone.left(x_epsilon * 100)

    def right(self):
        self.drone.right(x_epsilon * 100)

    def up(self):
        self.drone.up(z_epsilon * 100)

    def down(self):
        self.drone.down(z_epsilon * 100)

    def forward(self):
        self.drone.forward(y_epsilon * 100)

    def backward(self):
        self.drone.back(y_epsilon * 100)

    def _stop_stream(self):
        self.stream = False

    def _start_stream(self):
        self.drone.send_command('streamon')
        cap = cv2.VideoCapture('udp://' + self.drone.tello_ip + ':11111')
        self.stream = True
        while self.stream:
            ret, frame = cap.read()
            self.frame = frame
        cap.release()
        self.drone.send_command('streamoff')


RealTelloController()

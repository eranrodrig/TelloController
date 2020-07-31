import threading
from easytello import tello
import cv2
from Constants import *
import numpy as np


class RealTelloController:

    def __init__(self):
        self.frame = np.zeros([default_frame_size, default_frame_size], np.uint8)
        self.drone = tello.Tello(debug=False)
        self.video_thread = threading.Thread(target=self._start_stream, daemon=True)
        self.video_thread.start()

    def at_exit(self):
        self.drone.land()
        self.drone.socket.close()

    def take_off(self):
        self.drone.takeoff()

    def land(self):
        self.drone.land()

    def take_picture(self):
        return self.frame

    def cw(self):
        self.drone.cw(rotation_epsilon * 180)

    def ccw(self):
        self.drone.ccw(rotation_epsilon * 180)

    def left(self):
        self.drone.left(real_tello_epsilon)

    def right(self):
        self.drone.right(real_tello_epsilon)

    def up(self):
        self.drone.up(real_tello_epsilon)

    def down(self):
        self.drone.down(real_tello_epsilon)

    def forward(self):
        self.drone.forward(real_tello_epsilon)

    def backward(self):
        self.drone.back(real_tello_epsilon)

    def _stop_stream(self):
        self.stream = False

    def _start_stream(self):
        self.drone.send_command('streamon')
        cap = cv2.VideoCapture('udp://' + self.drone.tello_ip + ':11111')
        self.stream = True
        while self.stream:
            ret, frame = cap.read()
            self.frame = np.rot90(frame, k=2)
        cap.release()
        self.drone.send_command('streamoff')


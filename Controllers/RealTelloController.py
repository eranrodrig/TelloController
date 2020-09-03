import socket
import threading
from easytello import tello
import cv2
from Constants import *
import numpy as np


class RealTelloController:

    def __init__(self):
        self.frame = np.zeros([default_frame_size, default_frame_size], np.uint8)
        self.drone = tello.Tello(debug=False)
        self.drone.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.video_thread = threading.Thread(target=self._start_stream, daemon=True)
        self.video_thread.start()

    def at_exit(self):
        self.drone.land()
        self.drone.socket.close()

    def take_off(self):
        thread = threading.Thread(target=self.drone.takeoff, daemon=True)
        thread.start()
        self.drone.takeoff()

    def land(self):
        thread = threading.Thread(target=self.drone.land, daemon=True)
        thread.start()
        self.drone.land()

    def take_picture(self):
        return self.frame

    def cw(self):
        thread = threading.Thread(target=self.drone.cw, args=(rotation_epsilon * 180,), daemon=True)
        thread.start()

    def ccw(self):
        thread = threading.Thread(target=self.drone.ccw, args=(rotation_epsilon * 180,), daemon=True)
        thread.start()

    def left(self):
        thread = threading.Thread(target=self.drone.left, args=(real_tello_epsilon,), daemon=True)
        thread.start()

    def right(self):
        thread = threading.Thread(target=self.drone.right, args=(real_tello_epsilon,), daemon=True)
        thread.start()

    def up(self):
        thread = threading.Thread(target=self.drone.up, args=(real_tello_epsilon,), daemon=True)
        thread.start()

    def down(self):
        thread = threading.Thread(target=self.drone.down, args=(real_tello_epsilon,), daemon=True)
        thread.start()

    def forward(self):
        thread = threading.Thread(target=self.drone.forward, args=(real_tello_epsilon,), daemon=True)
        thread.start()

    def backward(self):
        thread = threading.Thread(target=self.drone.back, args=(real_tello_epsilon,), daemon=True)
        thread.start()


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


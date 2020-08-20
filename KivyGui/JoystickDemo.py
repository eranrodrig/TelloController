from time import sleep
import os
os.environ["KIVY_NO_CONSOLELOG"] = "1"
import cv2
from kivy.app import App
from garden.joystick import Joystick
from kivy.properties import ObjectProperty
from kivy.uix.image import Image
from kivy.uix.floatlayout import FloatLayout
from kivy.graphics.texture import Texture
from kivy.clock import Clock
import numpy as np
from threading import Thread


tello = None


def debounce(wait, debounced, args):
    def func():
        while True:
            debounced(args)
            sleep(wait)
    thread = Thread(target=func, daemon=True)
    thread.start()


class JoystickDemo(FloatLayout):
    left_joystick = ObjectProperty(None)
    right_joystick = ObjectProperty(None)
    wait = 0.2

    def takeoff(self):
        tello.take_off()

    def land(self):
        tello.land()

    def bind_joysticks(self):
        left_control = self._get_joystick(self.left_joystick)
        right_control = self._get_joystick(self.right_joystick)
        debounce(self.wait, self._left_joystick_control, left_control)
        debounce(self.wait, self._right_joystick_control, right_control)
        # self._get_joystick(self.left_joystick).bind(pad=self._left_joystick_control)
        # self._get_joystick(self.right_joystick).bind(pad=self._right_joystick_control)

    def _left_joystick_control(self, instance):
        angle = instance.angle
        if instance.magnitude > 0.9:
            if angle > 315 or angle < 45:
                tello.ccw()
            elif 45 < angle < 135:
                tello.up()
            elif 135 < angle < 225:
                tello.cw()
            else:
                tello.down()

    def _get_joystick(self, parent):
        if isinstance(parent, Joystick):
            return parent
        elif hasattr(parent, 'children'):
            for child in parent.children:
                if isinstance(child, Joystick):
                    return child

    def _right_joystick_control(self, instance):
        angle = instance.angle
        if instance.magnitude > 0.9:
            if angle > 315 or angle < 45:
                tello.left()
            elif 45 < angle < 135:
                tello.forward()
            elif 135 < angle < 225:
                tello.right()
            else:
                tello.backward()


class Video1(Image):
    fps = ObjectProperty(60)

    def __init__(self, **kwargs):
        Image.__init__(self, **kwargs)
        Clock.schedule_interval(self.update, 1.0 / self.fps)

    def update(self, dt):
        # frame = np.rot90(tello.take_picture(), k=2)
        frame = tello.take_picture()
        img = cv2.cvtColor(frame.astype(np.uint8), cv2.COLOR_BGR2RGB)
        image_texture = Texture.create(
            size=(img.shape[1], img.shape[0]), colorfmt='bgr')
        image_texture.blit_buffer(img.tostring(), colorfmt='bgr', bufferfmt='ubyte')
        # display image from the texture
        self.texture = image_texture


class JoystickDemoApp(App):

    def build(self):
        self.root = JoystickDemo()
        self.root.bind_joysticks()


def run_kv(proxy_tello):
    global tello
    tello = proxy_tello
    JoystickDemoApp().run()

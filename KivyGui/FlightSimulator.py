from time import sleep
from kivy.app import App
from kivy.garden.joystick import Joystick
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


class FlightSimulator(FloatLayout):
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


class VideoScreen(Image):
    fps = ObjectProperty(60)

    def __init__(self, **kwargs):
        Image.__init__(self, **kwargs)
        Clock.schedule_interval(self.update, 1.0 / self.fps)

    def update(self, dt):
        frame = tello.take_picture()
        img = frame.astype(np.uint8)
        image_texture = Texture.create(
            size=(img.shape[1], img.shape[0]))
        image_texture.blit_buffer(img.tostring())
        # display image from the texture
        self.texture = image_texture


class FlightSimulatorApp(App):

    def build(self):
        self.root = FlightSimulator()
        self.root.bind_joysticks()


def run_kv(proxy_tello):
    global tello
    tello = proxy_tello
    FlightSimulatorApp().run()

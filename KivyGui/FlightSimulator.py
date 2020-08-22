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
        pass

    def land(self):
        pass

    def bind_joysticks(self):
        left_control = self._get_joystick(self.left_joystick)
        right_control = self._get_joystick(self.right_joystick)
        debounce(self.wait, self._left_joystick_control, left_control)
        debounce(self.wait, self._right_joystick_control, right_control)

    def _left_joystick_control(self, instance):
        pass

    def _get_joystick(self, parent):
        if isinstance(parent, Joystick):
            return parent
        elif hasattr(parent, 'children'):
            for child in parent.children:
                if isinstance(child, Joystick):
                    return child

    def _right_joystick_control(self, instance):
        pass


class VideoScreen(Image):
    fps = ObjectProperty(60)

    def __init__(self, **kwargs):
        Image.__init__(self, **kwargs)
        Clock.schedule_interval(self.update, 1.0 / self.fps)

    def update(self, dt):
        pass


class FlightSimulatorApp(App):

    def build(self):
        self.root = FlightSimulator()
        self.root.bind_joysticks()


def run_kv(proxy_tello):
    global tello
    tello = proxy_tello
    FlightSimulatorApp().run()

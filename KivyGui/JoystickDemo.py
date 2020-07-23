import cv2
from kivy.app import App
from garden.joystick import Joystick
from kivy.properties import ObjectProperty
from kivy.uix.image import Image
from kivy.uix.floatlayout import FloatLayout
from kivy.graphics.texture import Texture
from kivy.clock import Clock
from TelloWebotsController import TelloWebotsController
import numpy as np


# tello = TelloWebotsController()
tello = None

class JoystickDemo(FloatLayout):
    def btn(self):
        tello.take_off()


class Video1(Image):
    fps = ObjectProperty(24)

    def __init__(self, **kwargs):
        Image.__init__(self, **kwargs)
        Clock.schedule_interval(self.update, 1.0 / self.fps)

    def update(self, dt):
        frame = np.rot90(tello.take_picture(), k=2)
        img = cv2.cvtColor(frame.astype(np.uint8), cv2.COLOR_BGR2RGB)
        image_texture = Texture.create(
            size=(img.shape[1], img.shape[0]), colorfmt='bgr')
        image_texture.blit_buffer(img.tostring(), colorfmt='bgr', bufferfmt='ubyte')
        # display image from the texture
        self.texture = image_texture


class JoystickDemoApp(App):
    def build(self):
        self.root = JoystickDemo()
        self._bind_joysticks()

    def _bind_joysticks(self):
        joysticks = self._get_joysticks(self.root)
        for joystick in joysticks:
            joystick.bind(pad=self._update_pad_display)

    def _get_joysticks(self, parent):
        joysticks = []
        if isinstance(parent, Joystick):
            joysticks.append(parent)
        elif hasattr(parent, 'children'):
            for child in parent.children:
                joysticks.extend(self._get_joysticks(child))
        return joysticks

    def _update_pad_display(self, instance, pad):
        return
    #     x, y = pad
    #     x, y = (str(x)[0:5], str(y)[0:5])
    #     x, y = (('x: ' + x), ('\ny: ' + y))
    #     r = "radians: " + str(instance.radians)[0:5]
    #     m = "\nmagnitude: " + str(instance.magnitude)[0:5]
    #     a = "\nangle: " + str(instance.angle)[0:5]
    #     self.root.ids.pad_display_xy.text = "".join([x, y])
    #     self.root.ids.pad_display_rma.text = "".join([r, m, a])


def run_kv(proxy_tello):
    global tello
    tello = proxy_tello
    JoystickDemoApp().run()

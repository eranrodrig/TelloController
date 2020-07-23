import multiprocessing
from multiprocessing.managers import BaseManager
from TelloWebotsController import TelloWebotsController
from KivyGui.JoystickDemo import run_kv


class MyManager(BaseManager):
    pass


def Manager():
    m = MyManager()
    m.start()
    return m


MyManager.register('TelloWebotsController', TelloWebotsController)


def main():
    manager = Manager()
    tello_webots_controller = manager.TelloWebotsController()
    pool = multiprocessing.Pool(1)
    pool.apply(func=run_kv, args=(tello_webots_controller,))
    pool.close()
    pool.join()

if __name__ == '__main__':
    main()

import atexit
import multiprocessing
from multiprocessing.managers import BaseManager

from RealTelloController import RealTelloController
from TelloWebotsController import TelloWebotsController
from KivyGui.JoystickDemo import run_kv


class MyManager(BaseManager):
    pass


def Manager():
    m = MyManager()
    m.start()
    return m


MyManager.register('TelloWebotsController', TelloWebotsController)
MyManager.register('RealTelloController', RealTelloController)


def main():
    manager = Manager()
    controller = manager.TelloWebotsController()
    pool = multiprocessing.Pool(1)
    pool.apply(func=run_kv, args=(controller,))
    pool.close()
    pool.join()
    try:
        controller.at_exit()
    except EOFError:
        pass


if __name__ == '__main__':
    main()

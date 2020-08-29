import multiprocessing
from multiprocessing.managers import BaseManager

from Controllers.RealTelloController import RealTelloController
from Controllers.TelloWebotsController import TelloWebotsController
from KivyGui.FlightSimulator import run_kv


class MyManager(BaseManager):
    pass


def Manager():
    m = MyManager()
    m.start()
    return m


MyManager.register('TelloWebotsController', TelloWebotsController)
MyManager.register('RealTelloController', RealTelloController)


def main():
    try:
        manager = Manager()
        controller = manager.RealTelloController()
        pool = multiprocessing.Pool(1)
        pool.apply(func=run_kv, args=(controller,))
        pool.close()
        pool.join()
        controller.at_exit()
    except EOFError:
        pass
    except Exception as e:
        print(e)
        controller.at_exit()
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()

import ctypes
import threading
import traceback

from behaviors.BasicBehaviors import BasicBehaviors
from robot.brain import brain
from robot.ui import SharedData
from ros import runros


def main():
    print("Starting Up...")

    behaviors = BasicBehaviors()
    shared = SharedData()

    # Start the ROS worker thread
    rosthread = threading.Thread(name="ROSThread", target=runros, args=(shared,))
    rosthread.start()

    print("Running...")

    try:
        brain(behaviors, shared)

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    print("Shutting Down...")

    # Kill the ROS thread
    ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(rosthread.ident), ctypes.py_object(KeyboardInterrupt))
    rosthread.join()

    behaviors.shutdown()


if __name__ == "__main__":
    main()

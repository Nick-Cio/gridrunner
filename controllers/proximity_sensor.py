# Groups three ultrasound sensors into left/middle/right
import pigpio
import time
import threading, prctl
import random

from hardware.ultrasound import Ultrasound

from robot.config import (
     ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO,
     ULTRA_MIDDLE_TRIG, ULTRA_MIDDLE_ECHO,
     ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO,
 )


class ProximitySensor:
    # Bundles the three ultrasound sensors

    def __init__(self, io):
        self.left = Ultrasound(io, ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO)
        self.middle = Ultrasound(io, ULTRA_MIDDLE_TRIG, ULTRA_MIDDLE_ECHO)
        self.right = Ultrasound(io, ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO)

        # Start a worker thread that keeps triggering on its own
        print("Starting the triggering thread...")
        self.triggering = True
        self.thread = threading.Thread(name="TriggerThread", target=self.run)
        self.thread.start()
        time.sleep(0.1)   # wait for the first measurements to arrive

    def trigger(self):
        # Fire all three sensors at once
        self.left.trigger()
        self.middle.trigger()
        self.right.trigger()

    def read(self):
        # Returns (left, middle, right) distances in meters
        return (self.left.read(), self.middle.read(), self.right.read())

    def run(self):
        prctl.set_name(threading.current_thread().name)
        # Triggering loop, runs in the worker thread.
        # Randomized intervals so we don't lock into a regular pattern.
        while self.triggering:
            self.trigger()
            time.sleep(random.uniform(0.047, 0.053))
            self.trigger()
            time.sleep(random.uniform(0.059, 0.065))

    def shutdown(self):
        # Stop the triggering thread and wait for it to return
        self.triggering = False
        print("Waiting for triggering thread to finish...")
        self.thread.join()
        print("Triggering thread returned.")


if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to the GPIO")
        exit()

    sensor = ProximitySensor(io)
    front = sensor.middle

    try:
        while True:
            print(f"d = {front.read():.2f} : "
                  f"<20cm = {front.blocked_below20cm()} ({front.samples20} samples) "
                  f"<65cm = {front.blocked_below65cm()} ({front.samples65} samples) "
                  f"<92cm = {front.blocked_below92cm()} ({front.samples92} samples)")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    sensor.shutdown()
    io.stop()

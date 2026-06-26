# Low-level ultrasound sensor reading via GPIO

import pigpio

from robot.config import SPEED_OF_SOUND, ULTRASOUND_INIT_DISTANCE


def cmp(a, b):
    return (a > b) - (b > a)


class Ultrasound:
    # Single ultrasound: pulses TRIGGER, times the ECHO pulse, stores distance in meters

    def __init__(self, io, pintrig, pinecho):
        self.io = io
        self.pintrig = pintrig
        self.pinecho = pinecho

        self.io.set_mode(pintrig, pigpio.OUTPUT)
        self.io.set_mode(pinecho, pigpio.INPUT)

        # Callbacks fire when ECHO rises (start of pulse) and falls (end of pulse)
        self.cbrise = io.callback(pinecho, pigpio.RISING_EDGE, self.rising)
        self.cbfall = io.callback(pinecho, pigpio.FALLING_EDGE, self.falling)

        self.risetick = 0
        self.distance = ULTRASOUND_INIT_DISTANCE

        # Bucketed obstacle detectors (3-of-5 style)
        self.samples20 = 0
        self.samples65 = 0
        self.samples92 = 0

    def trigger(self):
        # Pulse TRIGGER high for at least 10us, then back low.
        self.io.write(self.pintrig, 1)
        self.io.write(self.pintrig, 1)
        self.io.write(self.pintrig, 1)
        self.io.write(self.pintrig, 0)

    def rising(self, pin, level, ticks):
        # Remember when the echo pulse started
        self.risetick = ticks

    def falling(self, pin, level, ticks):
        # Echo ended: compute pulse width and convert to a distance
        deltatick = ticks - self.risetick
        if deltatick < 0:
            deltatick += 2 ** 32

        # 2 way trip in microseconds
        self.distance = deltatick * SPEED_OF_SOUND / 2 / 1_000_000

        # bump each bucket up if obstacle is closer than its threshold, down otherwise
        self.samples20 = max(0, min(5, self.samples20 + cmp(0.35, self.distance)))
        self.samples65 = max(0, min(5, self.samples65 + cmp(0.65, self.distance)))
        self.samples92 = max(0, min(5, self.samples92 + cmp(0.92, self.distance)))

    def read(self):
        return self.distance

    def blocked_below20cm(self):
        return self.samples20 > 3

    def blocked_below65cm(self):
        return self.samples65 > 3

    def blocked_below92cm(self):
        return self.samples92 > 3

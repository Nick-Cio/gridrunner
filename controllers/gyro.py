from hardware.adc import ADC
from robot.config import (
    GYRO_SCALE,
    GYRO_RESETZERO_SAMPLES,
    GYRO_RESETZERO_INTERVAL_S,
)
import time

class Gyroscope():
    def __init__(self, io):
        self.io = io
        self.scale = GYRO_SCALE
        self.adc = ADC(io)

    def resetzero(self):
        readings = []
        for i in range(GYRO_RESETZERO_SAMPLES):
            readings.append(self.adc.read())
            time.sleep(GYRO_RESETZERO_INTERVAL_S)

        offset = sum(readings) / len(readings)

        return offset

    def read(self, offset):
        reading = self.adc.read()
        omega = self.scale * (reading - offset)
        return omega

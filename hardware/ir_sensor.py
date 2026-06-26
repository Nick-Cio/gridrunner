# Low-level infrared sensor reading via GPIO

import pigpio


class IR:
    #Reads a single IR reflectance sensor on a GPIO pin

    def __init__(self, gpio, io):
        self.io = io
        self.pin = gpio
        self.io.set_mode(gpio, pigpio.INPUT)

    def read(self):
        #Return the current digital reading (0 or 1) from the sensor
        return self.io.read(self.pin)

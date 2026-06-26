import pigpio
import time

from robot.config import ADC_CLOCK_PIN, ADC_BIT_PINS, ADC_READ_DELAY_S


class ADC():
    def __init__(self, io):
        self.io = io
        self.io.set_mode(ADC_CLOCK_PIN, pigpio.OUTPUT)
        for pin in ADC_BIT_PINS:
            self.io.set_mode(pin, pigpio.INPUT)

    def read(self):
        self.io.write(ADC_CLOCK_PIN, 0)
        self.io.write(ADC_CLOCK_PIN, 0)
        self.io.write(ADC_CLOCK_PIN, 1)
        last_time = time.time()
        while time.time() - last_time < ADC_READ_DELAY_S:
            pass
        
        Bit7 = self.io.read(ADC_BIT_PINS[0])
        Bit6 = self.io.read(ADC_BIT_PINS[1])
        Bit5 = self.io.read(ADC_BIT_PINS[2])
        Bit4 = self.io.read(ADC_BIT_PINS[3])
        Bit3 = self.io.read(ADC_BIT_PINS[4])
        Bit2 = self.io.read(ADC_BIT_PINS[5])
        Bit1 = self.io.read(ADC_BIT_PINS[6])
        Bit0 = self.io.read(ADC_BIT_PINS[7])

        adc_output = ((2**7)*Bit7) + ((2**6)*Bit6) + ((2**5)*Bit5) + ((2**4)*Bit4) + ((2**3)*Bit3) + ((2**2)*Bit2) + ((2**1)*Bit1) + ((2**0)*Bit0)

        return adc_output


############# test
"""
io =pigpio.pi()

adc = ADC(io)

while True:
    print(adc.read())
"""
"""
a) Holding the pi/robot still. What integer value does the A/D report?

- 135/4

(b) Spinning up/down. What is the highest and lowest A/D values you can read?

- 140/120


(c) Spinning CCW at about 1 revolution in 4 seconds (π/2 rad/sec). Approximately what value
do you read?

- 150

(d) Spinning CW in the opposite direction, again at about 1/4 rev/sec (π/2 rad/sec). In this
direction, approximately what value do you read?

- 120

(e) From this, what is the approximate scale and offset to compute
omega = scale * (adc.read() - offset) for omega in rad/sec?

offset = 135

scale = 0.1

"""
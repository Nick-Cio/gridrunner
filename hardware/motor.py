# Low-level motor control via PWM on GPIO pins.

import pigpio
from robot.config import PWM_RANGE, PWM_FREQUENCY


class Motor:
    #Controls a single DC motor through an H-bridge using two GPIO pins

    def __init__(self, f_pin, b_pin, io):
        self.f_pin = f_pin
        self.b_pin = b_pin
        self.io = io

        # Configure both pins as PWM outputs.
        self.io.set_mode(self.f_pin, pigpio.OUTPUT)
        self.io.set_mode(self.b_pin, pigpio.OUTPUT)

        self.io.set_PWM_range(self.f_pin, PWM_RANGE)
        self.io.set_PWM_range(self.b_pin, PWM_RANGE)

        self.io.set_PWM_frequency(self.f_pin, PWM_FREQUENCY)
        self.io.set_PWM_frequency(self.b_pin, PWM_FREQUENCY)

        # Initialize both pins high (braked state).
        self.io.set_PWM_dutycycle(self.b_pin, PWM_RANGE)
        self.io.set_PWM_dutycycle(self.f_pin, PWM_RANGE)

    def off(self):
        #Cut all power to the motor (coast to stop)
        self.io.set_PWM_dutycycle(self.f_pin, 0)
        self.io.set_PWM_dutycycle(self.b_pin, 0)

    def stop(self):
        #Actively brake the motor (both pins high)
        self.io.set_PWM_dutycycle(self.f_pin, PWM_RANGE)
        self.io.set_PWM_dutycycle(self.b_pin, PWM_RANGE)

    def set_level(self, level):
        """
        Set motor speed and direction.
        Input: float [-1, 1]; positive = forward, negative = reverse.
        """
        abs_level = abs(level)
        pwm_dutycycle = int(abs_level * PWM_RANGE)
        inverted_pwm_dutycycle = PWM_RANGE - pwm_dutycycle

        if level > 0:
            self.io.set_PWM_dutycycle(self.f_pin, PWM_RANGE)
            self.io.set_PWM_dutycycle(self.b_pin, inverted_pwm_dutycycle)
        else:
            self.io.set_PWM_dutycycle(self.f_pin, inverted_pwm_dutycycle)
            self.io.set_PWM_dutycycle(self.b_pin, PWM_RANGE)

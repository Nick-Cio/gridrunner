# Coordinates left and right motors into a drive system

from hardware.motor import Motor
from robot.config import (
    MOTOR_R_FORWARD_PIN, MOTOR_R_BACKWARD_PIN,
    MOTOR_L_FORWARD_PIN, MOTOR_L_BACKWARD_PIN,
    DEGREE_LEVEL_RIGHT, DEGREE_LEVEL_LEFT,
    PULL_FORWARD_LEFT_LEVEL, PULL_FORWARD_RIGHT_LEVEL,
)
import time


class DriveSystem:
    #Drive controller wrapping two motors

    def __init__(self, io):
        self.right_motor = Motor(MOTOR_R_FORWARD_PIN, MOTOR_R_BACKWARD_PIN, io)
        self.left_motor = Motor(MOTOR_L_FORWARD_PIN, MOTOR_L_BACKWARD_PIN, io)

    def drive(self, degree, direction, reverse=False):
        """
        Drive the robot with a given turning degree and direction.

        Input:
            degree: steepness of turn (0-5)
            direction: 1 right, -1 left
            reverse: if True, negate the PWM (back up while keeping the same turn shape)

        Output:
            Tuple (left_level, right_level) of what was applied/sent to the motors
        """
        if direction == 1:  # Turn right
            levels = DEGREE_LEVEL_RIGHT[degree]
        else:  # Turn left
            levels = DEGREE_LEVEL_LEFT[degree]

        if reverse:
            sign = -1
        else:
            sign = 1
        
        left = sign * levels[0]
        right = sign * levels[1]
        self.left_motor.set_level(left)
        self.right_motor.set_level(right)
        return (left, right)

    def pwm(self, pwm_l, pwm_r):
        """
        Directly set the two motor levels, bypassing the discrete degree table.
        """
        pwm_l = max(-1.0, min(1.0, pwm_l))
        pwm_r = max(-1.0, min(1.0, pwm_r))
        self.left_motor.set_level(pwm_l)
        self.right_motor.set_level(pwm_r)
        return (pwm_l, pwm_r)

    def brake(self):
        #Actively brake both motors
        self.left_motor.stop()
        self.right_motor.stop()

    def deactivate(self):
        #Cut power to both motors
        self.left_motor.off()
        self.right_motor.off()
    
    def pull_forward(self, run_time):
        center_time = run_time
        global_timer = time.time()
        while time.time() - global_timer < center_time:
            self.left_motor.set_level(PULL_FORWARD_LEFT_LEVEL)
            self.right_motor.set_level(PULL_FORWARD_RIGHT_LEVEL)
        self.brake()



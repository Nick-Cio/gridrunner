# Groups three IR sensors into a line-detection array

from hardware.ir_sensor import IR
from robot.config import (
    PIN_IR_LEFT, PIN_IR_MIDDLE, PIN_IR_RIGHT,
    SIDE_DETECTOR_THRESHOLD,
)


class LineSensors:
    #Reads a left/middle/right IR sensor array for line tracking

    def __init__(self, io):
        self.left = IR(PIN_IR_LEFT, io)
        self.middle = IR(PIN_IR_MIDDLE, io)
        self.right = IR(PIN_IR_RIGHT, io)

    def read_x3(self):
        """
        Read all three sensors.

        Output:
            Tuple of (left, middle, right); 0=white, 1=black.
        """
        return (
            self.left.read(),
            self.middle.read(),
            self.right.read(),
        )


# --- Binary event detectors ---
# Shared EMA update math.
# Each subclass differs based on the activation condition.

class BinaryEventDetector:
    # Returns True when accumulated evidence crosses the threshold.

    def __init__(self, T, threshold):
        self.state = 0.0        # initial val. is 0  
        self.T = T                
        self.threshold = threshold

    def update(self, reading, dt):
        # Based on unique condition choose value
        if self.condition(reading):
            target = 1
        else:
            target = 0

        # Update formula
        self.state += (dt / self.T) * (target - self.state)

        # Fire if above threshold
        if self.state >= self.threshold:
            return True
        else:
            return False


class IntersectionDetector(BinaryEventDetector):
    # Fires when all three sensors see black
    def condition(self, reading):
        if reading == (1, 1, 1):
            return True
        else:
            return False


class TurnDetector(BinaryEventDetector):
    # Fires when the sensor on the turn side sees black.
    # dir = 1: watches right sensor, dir = -1: watches left sensor, need to overide init
    def __init__(self, T, threshold, dir):
        self.state = 0.0
        self.T = T
        self.threshold = threshold
        self.dir = dir

    def condition(self, reading):
        if self.dir == 1:
            if reading[2] == 1:
                return True
            else:
                return False
        else:
            if reading[0] == 1:
                return True
            else:
                return False


class EndDetector(BinaryEventDetector):
    # Fires when all three sensors see white
    def condition(self, reading):
        if reading == (0, 0, 0):
            return True
        else:
            return False


class MiddleEndDetector(BinaryEventDetector):
    # Fires when the middle sensor sees white, regardless of left/right
    def condition(self, reading):
        if reading[1] == 0:
            return True
        else:
            return False


# --- Side-state tracker (tristate) ---
# Returns -1 / 0 / 1 to indicate drift direction.

class SideDetector:
    def __init__(self, state, T):
        self.state = state
        self.T = T

    def side_detector(self, sensor_reading, dt):

        STATE_LEVEL = {
        (0,1,1): 0.5,
        (0,0,1): 1,
        (0,1,0): 0,
        (1,1,0): -0.5,
        (1,0,0): -1,
        (1,0,1): 0,
        (1,1,1): 0,
        }

        if sensor_reading in STATE_LEVEL:
            self.state += (dt/self.T)*(STATE_LEVEL[sensor_reading]-self.state)

        if self.state >= SIDE_DETECTOR_THRESHOLD:
            return 1
        elif self.state <= -SIDE_DETECTOR_THRESHOLD:
            return -1
        else:
            return 0

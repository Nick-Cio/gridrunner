import pigpio
import sys
import time
import math
from controllers.drive_system import DriveSystem
from controllers.line_sensors import (
    LineSensors,
    IntersectionDetector, EndDetector, MiddleEndDetector, SideDetector, TurnDetector,
)
from robot.config import (
    SENSOR_ACTION,
    SIDE_DETECTOR_T,
    INTERSECTION_DETECTOR_T, INTERSECTION_DETECTOR_THRESHOLD,
    END_DETECTOR_T, END_DETECTOR_THRESHOLD,
    TURN_DETECTOR_T, TURN_DETECTOR_THRESHOLD,
    TURN_OFF_LINE_T, TURN_OFF_LINE_THRESHOLD,
    TURN_TIMEOUT_S, TURN_OVERSHOOT_S,
    INTERSECTION_PAUSE_S, INTERSECTION_PULL_FORWARD_S,
    PULL_FORWARD_LEFT_LEVEL, PULL_FORWARD_RIGHT_LEVEL,
    SPIRAL_FAST_LEVEL, SPIRAL_SLOW_LEVEL_INITIAL,
    SPIRAL_STEP_INTERVAL_S, SPIRAL_SLOW_INCREMENT, SPIRAL_MIN_LEVEL_GAP,
    GYRO_RESET_SETTLE_S,
    DEADEND_BRAKE_PAUSE_S,
)
from controllers.gyro import Gyroscope
from controllers.proximity_sensor import ProximitySensor
from controllers.nfc import NFCSensor


class BasicBehaviors:
    def __init__(self):
        self.io = pigpio.pi()
        if not self.io.connected:
            raise SystemError("Unable to connect to the GPIO")


        self.drive = DriveSystem(self.io)
        self.sensor = LineSensors(self.io)
        self.gyro = Gyroscope(self.io)
        self.proximity = ProximitySensor(self.io)
        self.nfc = NFCSensor()

        # remember the last turn so a same-direction follow-up to a 180 can recover
        self.prev_turn_raw = 0
        self.prev_turn_dir = 0
        self.prev_turn_slots = 0

    def shutdown(self):
        self.drive.brake()
        self.proximity.shutdown()
        self.nfc.shutdown()
        self.drive.deactivate()
        self.io.stop()

    def check_blockages(self):
        # Returns (left, front, right), each a tuple of (below20cm, below65cm, below92cm).
        left = self.proximity.left
        middle = self.proximity.middle
        right = self.proximity.right
        return (
            (left.blocked_below20cm(),   left.blocked_below65cm(),   left.blocked_below92cm()),
            (middle.blocked_below20cm(), middle.blocked_below65cm(), middle.blocked_below92cm()),
            (right.blocked_below20cm(),  right.blocked_below65cm(),  right.blocked_below92cm()),
        )

    def compound_start(self):
        # Compound street-follow used at startup: drive to the first intersection
        # Line-follow forward; if we hit a dead end, U-turn left and line-follow
        # again, and then finally pullthrough across the intersection

        u_turned = False
        result = self.follow_line()
        # Obstacle on the first segment: wait for it to clear
        while result == "FOUND_BLOCKAGE":
            while self.proximity.middle.blocked_below20cm():
                self.drive.brake()
                time.sleep(0.05)
            result = self.follow_line()
        if result == "FOUND_END":
            u_turned = True
            self.turn(-1)
            self.drive.brake()
            time.sleep(DEADEND_BRAKE_PAUSE_S)
            self.follow_line()
        road_ahead = self.pullthrough()
        return (u_turned, road_ahead)

    def search(self):
        # Drive in a spiral (one motor faster than the other) until any sensor detects black
        print("Spiral: searching for line")
        global_timer = time.time()
        fast_level = SPIRAL_FAST_LEVEL
        slow_level = SPIRAL_SLOW_LEVEL_INITIAL

        while True:
            self.drive.left_motor.set_level(fast_level)
            self.drive.right_motor.set_level(slow_level)

            # every 1 sec make the decrement smaller between the motors
            if time.time() - global_timer >= SPIRAL_STEP_INTERVAL_S:
                global_timer = time.time()
                slow_level = min(slow_level + SPIRAL_SLOW_INCREMENT, fast_level - SPIRAL_MIN_LEVEL_GAP)

            reading = self.sensor.read_x3()
            if reading != (0, 0, 0):
                print("Line found: braking")
                self.drive.brake()
                return "FOUND"

    def follow_line(self):
        # Line following with EMA-based recovery
        global_timer = time.time()

        # initialize detectors
        side_detector = SideDetector(0, SIDE_DETECTOR_T)
        intersection_detector = IntersectionDetector(INTERSECTION_DETECTOR_T, INTERSECTION_DETECTOR_THRESHOLD)
        end_detector = EndDetector(END_DETECTOR_T, END_DETECTOR_THRESHOLD)

        while True:
            # time step: full previous iteration time
            now = time.time()
            dt = now - global_timer
            global_timer = now

            # bail if something is in front of us — third exit condition
            if self.proximity.middle.blocked_below20cm():
                print("Obstacle in front: braking")
                self.drive.brake()
                return "FOUND_BLOCKAGE"

            # output of all 3 IR sensors
            sensor_reading = self.sensor.read_x3()

            # update values of detectors using EMA
            side_state = side_detector.side_detector(sensor_reading, dt)
            intersection = intersection_detector.update(sensor_reading, dt)
            end = end_detector.update(sensor_reading, dt)

            # intersection detector fires
            if intersection:
                print("Intersection detected: braking")
                self.drive.brake()
                time.sleep(INTERSECTION_PAUSE_S)
                return "FOUND_INTERSECTION"

            # drive pattern (straight or turning w/ degree) depends on sensor state
            if sensor_reading in SENSOR_ACTION:
                degree, direction = SENSOR_ACTION[sensor_reading]
                self.drive.drive(degree, direction)

            # either got pushed off or reached the end, react accordingly
            elif sensor_reading == (0, 0, 0):
                if side_state != 0:
                    if side_state == 1:
                        self.drive.drive(5, 1)
                    elif side_state == -1:
                        self.drive.drive(5, -1)
                    else:
                        print("Impossible side state reached")
                else:
                    if end:
                        print("End of road detected: braking")
                        self.drive.brake()
                        return "FOUND_END"
                    self.drive.drive(0, 1)

            else:
                print("Impossible sensor state reached")

    def turn(self, dir, threshold=None):
        # Two-phase:
        #   1. off_line=False: spin while off_line_det builds confidence we've left the line
        #   2. off_line=True:  feed turn_det until it fires on the new line, then stop
        # threshold defaults to config; pass a value to tune it live (flower_power).
        if threshold is None:
            threshold = TURN_DETECTOR_THRESHOLD
        turn_det = TurnDetector(TURN_DETECTOR_T, threshold, dir)
        off_line_det = EndDetector(TURN_OFF_LINE_T, TURN_OFF_LINE_THRESHOLD)
        
        off_line = False

        self.drive.brake()
        time.sleep(GYRO_RESET_SETTLE_S)
        offset = self.gyro.resetzero()
        time.sleep(GYRO_RESET_SETTLE_S)

        angle_accumulated = 0

        if dir == 1:
            direction_label = "right"
        else:
            direction_label = "left"
        print(f"Turning {direction_label}")
        
        turn_start = time.time()
        global_timer = time.time()

        while True:
            if time.time() - turn_start > TURN_TIMEOUT_S:
                print("Turn timeout: braking")
                self.drive.brake()
                self.prev_turn_slots = 0
                return None
          
            now = time.time()
            dt = now - global_timer                                                                                                                                                                              
            global_timer = now                                
                                                
            # read gyro and accumulate angle every loop iteration
            omega = self.gyro.read(offset)       # rad/s
            angle_accumulated += omega * dt      # small slice of angle each step                                                                                                                                
                                                
            sensor_reading = self.sensor.read_x3()

            if not off_line:
                # Phase 1: wait for sustained all-white before transitioning
                if off_line_det.update(sensor_reading, dt):
                    off_line = True
            else:
                # Phase 2: feed sensor reads into the EMA-based turn detector
                if turn_det.update(sensor_reading, dt):
                    print("Line found: braking")
                    self.drive.brake()
                    raw_deg = angle_accumulated * 180 / math.pi
                    measured = abs(raw_deg)
                    raw_abs = measured
                    # Patch: a same-direction turn right after a 180 that snaps to 90
                    # (likely a 135 shortened by the 180's over-commit); add back 0.75 of it.
                    if self.prev_turn_slots == 4 and dir == self.prev_turn_dir and round(raw_abs / 45) == 2:
                        raw_abs += 0.75 * (self.prev_turn_raw - 180)
                    # we only get here after finding a new line, so we turned at least 45
                    slots = max(1, round(raw_abs / 45))
                    angle_deg = slots * 45 * -1 * dir
                    print(f"[turn] raw={raw_deg:+.1f}  snapped={angle_deg:+d}  ({angle_deg // 45 if angle_deg else 0:+d} slot{'s' if abs(angle_deg) != 45 else ''})")

                    # remember this turn (real measurement + snapped slots) for the next call
                    self.prev_turn_raw = measured
                    self.prev_turn_dir = dir
                    self.prev_turn_slots = slots

                    return angle_deg

            self.drive.drive(5, dir)

    def pullthrough(self, run_time=None):
        # Pull forward across the intersection while running an EMA end-of-road
        # (middle-sensor-only: ignores L/R so cross-bar of intersection doesn't matter)
        # run_time defaults to config; pass a value to tune it live (flower_power).
        if run_time is None:
            run_time = INTERSECTION_PULL_FORWARD_S
        end_detector = MiddleEndDetector(END_DETECTOR_T, END_DETECTOR_THRESHOLD)

        start = time.time()
        last_loop = start
        road_ahead = True

        while time.time() - start < run_time:
            now = time.time()
            dt = now - last_loop
            last_loop = now

            self.drive.left_motor.set_level(PULL_FORWARD_LEFT_LEVEL)
            self.drive.right_motor.set_level(PULL_FORWARD_RIGHT_LEVEL)

            sensor_reading = self.sensor.read_x3()
            if end_detector.update(sensor_reading, dt):
                road_ahead = False

        self.drive.brake()
        return road_ahead

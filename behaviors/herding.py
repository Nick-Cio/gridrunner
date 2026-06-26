# Herding behavior: the three ultrasound sensors repulse the robot

import time
import pigpio

from controllers.drive_system import DriveSystem
from controllers.proximity_sensor import ProximitySensor
from robot.config import (
    HERD_TRIGGER_INTERVAL_S,
    HERD_SIDE_THRESHOLD_M,
    HERD_FRONT_CLEAR_M,
    HERD_FRONT_BACKUP_M,
    HERD_TURN_DEGREE,
    HERD_SPIN_DEGREE,
)


# Forward motion state from the middle sensor
ADVANCE = "advance"
HOLD = "hold"
BACKUP = "backup"


def forward_state(d_middle):
    # Three-way bucket from the front distance
    if d_middle > HERD_FRONT_CLEAR_M:
        return ADVANCE
    if d_middle < HERD_FRONT_BACKUP_M:
        return BACKUP
    return HOLD


def choose_action(d_left, d_middle, d_right):
    """
    Map the 3 distance buckets x 4 side combinations to a drive command.

    Returns either None (= brake) or (degree, direction, reverse).
    """
    state = forward_state(d_middle)
    left_blocked = d_left < HERD_SIDE_THRESHOLD_M
    right_blocked = d_right < HERD_SIDE_THRESHOLD_M

    if state == ADVANCE:
        if left_blocked and not right_blocked:
            # Obstacle on left -> turn right while moving forward
            return (HERD_TURN_DEGREE, 1, False)
        if right_blocked and not left_blocked:
            # Obstacle on right -> turn left while moving forward
            return (HERD_TURN_DEGREE, -1, False)
        # Neither side blocked, or symmetric: just drive straight
        return (0, 1, False)

    if state == HOLD:
        if left_blocked and not right_blocked:
            return (HERD_SPIN_DEGREE, 1, False)   # spin right (away from left)
        if right_blocked and not left_blocked:
            return (HERD_SPIN_DEGREE, -1, False)  # spin left (away from right)
        return None

    # state == BACKUP
    if left_blocked and not right_blocked:
        # Back up while rotating CW (rear curves right, away from left obstacle)
        return (HERD_TURN_DEGREE, -1, True)
    if right_blocked and not left_blocked:
        # Back up while rotating CCW (rear curves left, away from right obstacle)
        return (HERD_TURN_DEGREE, 1, True)
    return (0, 1, True)


def apply_action(drive, action):
    if action is None:
        drive.brake()
    else:
        degree, direction, reverse = action
        drive.drive(degree, direction, reverse=reverse)


def herd(drive, sensor):
    # Repulsion loop: trigger every 50ms, react on the latest reading
    sensor.trigger()
    last_trigger_time = time.time()

    while True:
        now = time.time()
        if now - last_trigger_time > HERD_TRIGGER_INTERVAL_S:
            sensor.trigger()
            last_trigger_time = now

        d_left, d_middle, d_right = sensor.read()
        action = choose_action(d_left, d_middle, d_right)
        print(f"d=({d_left:.2f}, {d_middle:.2f}, {d_right:.2f}) -> {action}")
        apply_action(drive, action)


def main():
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to the GPIO")
        return

    drive = DriveSystem(io)
    sensor = ProximitySensor(io)

    try:
        herd(drive, sensor)
    except KeyboardInterrupt:
        pass
    finally:
        drive.brake()
        drive.deactivate()
        io.stop()


if __name__ == "__main__":
    main()

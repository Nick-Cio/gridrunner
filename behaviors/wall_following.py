# Wall following behavior on left side

import pigpio
import threading

from controllers.drive_system import DriveSystem
from controllers.proximity_sensor import ProximitySensor
from robot.config import (
    WALL_NOMINAL_DISTANCE_M,
    WALL_STOP_ERROR_M,
    WALL_FRONT_STOP_M,
    WALL_C_L, WALL_K_L,
    WALL_C_R, WALL_K_R,
)


# Shared data passed between the UI thread and the main wall-following loop.
# mode values:  -1 quit, 0 stop, 1 discrete, 2 continuous

class SharedData:
    def __init__(self):
        self.lock = threading.Lock()
        self.mode = 0   # start stopped, wait for the user

    def acquire(self):
        return self.lock.acquire()

    def release(self):
        self.lock.release()


def compute_error(sensor):
    # Returns the left-wall error in meters, or None if we should brake.
    d_left, d_middle, d_right = sensor.read()

    # Obstacle straight ahead overrides everything else
    if d_middle <= WALL_FRONT_STOP_M:
        print(f"Obstacle ahead at {d_middle:.2f}m")
        return None

    error = d_left - WALL_NOMINAL_DISTANCE_M

    # Outside the recoverable window
    if abs(error) > WALL_STOP_ERROR_M:
        print(f"Out of range (d_left={d_left:.2f}m, err={error:+.2f})")
        return None

    return error


# ---- continuous (proportional PWM) ----

def compute_pwm(error):
    pwm_l = WALL_C_L + WALL_K_L * error
    pwm_r = WALL_C_R + WALL_K_R * error
    return (pwm_l, pwm_r)


def drive_continuous(error, drive):
    pwm_l, pwm_r = compute_pwm(error)
    applied_l, applied_r = drive.pwm(pwm_l, pwm_r)
    print(f"err={error:+.2f}  pwm=({applied_l:.2f}, {applied_r:.2f})")


# ---- discrete (error-band) ----

def drive_discrete(error, drive):
    abs_error = abs(error)

    # Sign of error picks the direction
    if error > 0:
        direction = -1
    else:
        direction = 1

    # Magnitude picks how sharp
    if abs_error < 0.02:
        degree = 1
    elif abs_error < 0.05:
        degree = 2
    elif abs_error < 0.08:
        degree = 3
    else:
        degree = 4

    drive.drive(degree, direction)
    print(f"err={error:+.2f}  drive=({degree}, {direction})")


def follow_wall(drive, sensor, shared):
    while True:
        # Grab the latest mode (copy it then release the lock)
        if shared.acquire():
            mode = shared.mode
            shared.release()

        if mode < 0:
            break

        if mode == 0:
            drive.brake()
            continue

        error = compute_error(sensor)
        if error is None:
            drive.brake()
            continue

        if mode == 1:
            drive_discrete(error, drive)
        elif mode == 2:
            drive_continuous(error, drive)


def runui(shared):
    # Daemon UI loop, dies automatically when main exits.
    while True:
        cmd = input("Command: Quit (-1), Stop (0), Discrete (1), Continuous (2): ")
        cmd = cmd.strip()

        try:
            mode = int(cmd)
        except ValueError:
            print("Invalid input buddy.")
            continue

        if mode not in (-1, 0, 1, 2):
            print("Invalid input buddy.")
            continue

        if shared.acquire():
            shared.mode = mode
            shared.release()

        if mode == -1:
            return


def main():
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to the GPIO")
        return

    drive = DriveSystem(io)
    sensor = ProximitySensor(io)
    shared = SharedData()

    # Start the UI worker thread as a daemon so it auto-kills on exit
    uithread = threading.Thread(name="UIThread", target=runui, args=(shared,))
    uithread.daemon = True
    uithread.start()

    try:
        follow_wall(drive, sensor, shared)
    except KeyboardInterrupt:
        pass
    finally:
        drive.brake()
        sensor.shutdown()
        drive.deactivate()
        io.stop()


if __name__ == "__main__":
    main()

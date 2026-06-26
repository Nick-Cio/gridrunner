# Calibration test: live-tune the turn-detector threshold and the pull-through time.
# Run from the goals9 dir:  python3 -m tests.flower_power

import sys
import time
import traceback

from behaviors.BasicBehaviors import BasicBehaviors
from robot.config import TURN_DETECTOR_THRESHOLD, INTERSECTION_PULL_FORWARD_S


if __name__ == "__main__":
    behaviors = BasicBehaviors()

    threshold = TURN_DETECTOR_THRESHOLD
    pull_time = INTERSECTION_PULL_FORWARD_S
    THR_STEP = 0.05
    PULL_STEP = 0.1

    print("Tuner:")
    print("  R / L : turn right / left at the current threshold")
    print("  + / - : raise / lower turn threshold by %.2f   (number sets it, e.g. 0.55)" % THR_STEP)
    print("  P     : pull-through forward for the current time (watch where it stops)")
    print("  ] / [ : raise / lower pull-through time by %.1fs" % PULL_STEP)
    print("  D     : drive forward toggle      Q : quit")

    try:
        driving = False
        while True:
            cmd = input("[thr=%.2f  pull=%.2fs] R/L P ]/[ +/- D Q: " % (threshold, pull_time)).strip().upper()

            if cmd == "Q":
                break
            elif cmd == "R":
                behaviors.turn(1, threshold=threshold)
            elif cmd == "L":
                behaviors.turn(-1, threshold=threshold)
            elif cmd == "P":
                print("Pull-through for %.2fs..." % pull_time)
                behaviors.pullthrough(run_time=pull_time)
            elif cmd == "+":
                threshold = min(0.95, threshold + THR_STEP)
                print("threshold -> %.2f" % threshold)
            elif cmd == "-":
                threshold = max(0.10, threshold - THR_STEP)
                print("threshold -> %.2f" % threshold)
            elif cmd == "]":
                pull_time = min(3.0, pull_time + PULL_STEP)
                print("pull time -> %.2fs" % pull_time)
            elif cmd == "[":
                pull_time = max(0.10, pull_time - PULL_STEP)
                print("pull time -> %.2fs" % pull_time)
            elif cmd == "D":
                if driving:
                    behaviors.drive.brake()
                    driving = False
                    print("Stopped.")
                else:
                    behaviors.drive.drive(0, 1)   # degree 0 = straight forward
                    driving = True
                    print("Driving forward... (D to stop)")
            else:
                # a bare number sets the turn threshold (0.55 or 55 both mean 0.55)
                try:
                    val = float(cmd)
                    if val > 1:
                        val = val / 100
                    threshold = max(0.10, min(0.95, val))
                    print("threshold -> %.2f" % threshold)
                except ValueError:
                    print("Enter R, L, P, ], [, +, -, a number, D, or Q.")

        print("Done. threshold=%.2f (TURN_DETECTOR_THRESHOLD)  pull=%.2fs (INTERSECTION_PULL_FORWARD_S)"
              % (threshold, pull_time))

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    behaviors.shutdown()

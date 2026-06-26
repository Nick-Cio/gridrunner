# UI thread + shared data between the brain and the user
# The brain reads this state at every intersection

import threading, prctl
from enum import Enum


class Mode(Enum):
    MANUAL = "manual"
    EXPLORE = "explore"
    GOTO = "goto"
    ZONE = "zone"


class Manual(Enum):
    LEFT = "left"
    RIGHT = "right"
    STRAIGHT = "straight"


class SharedData:
    def __init__(self):
        self.lock = threading.Lock()

        self.quit = False
        self.stop = False

        self.mode = Mode.MANUAL

        # Goal coords used by Mode.GOTO
        self.xgoal = None
        self.ygoal = None

        # Zone bounds used by Mode.ZONE
        self.zone_xmin = None
        self.zone_xmax = None
        self.zone_ymin = None
        self.zone_ymax = None
        self.zone_treach = None

        # Latest loot dictionary from ROS: id -> {distance, name}
        self.loot = {}

        # One-shot manual action. Brain executes and sets back to None.
        self.manual_command = None

        # Empty = no save pending. Brain saves and resets to "".
        self.save_filename = ""

        # One-shot blockage clear
        self.clear = False

        # One-shot pose reset. Brain applies (x, y, h) and sets back to None.
        self.pose = None

        # Copy of the bot's current pose.
        self.robotx = 0
        self.roboty = 0
        self.robotheading = 0
        self.intersectionid = None

    def acquire(self):
        return self.lock.acquire()

    def release(self):
        self.lock.release()


PROMPT = ("Enter command: "
          "E (explore) G (goto) S (stop) "
          "L (left) R (right) F (forward) "
          "Z (zone) P (pose) V (save) C (clear) Q (quit): ")


def runui(shared):
    # Expose the thread name to Linux.
    prctl.set_name(threading.current_thread().name)
    while True:
        cmd = input(PROMPT).strip().upper()

        if cmd == "E":
            if shared.acquire():
                shared.mode = Mode.EXPLORE
                shared.release()

        elif cmd == "G":
            # second prompt for the actual goal coords
            coords = input("Enter goal X Y: ").strip().split()
            try:
                xg = int(coords[0])
                yg = int(coords[1])
            except (ValueError, IndexError):
                print("Invalid coords buddy.")
                continue
            if shared.acquire():
                shared.mode = Mode.GOTO
                shared.xgoal = xg
                shared.ygoal = yg
                shared.release()

        elif cmd == "S":
            if shared.acquire():
                shared.stop = True
                shared.release()

        elif cmd == "L":
            # a manual action implicitly switches to manual mode
            if shared.acquire():
                shared.mode = Mode.MANUAL
                shared.manual_command = Manual.LEFT
                shared.release()

        elif cmd == "R":
            # a manual action implicitly switches to manual mode
            if shared.acquire():
                shared.mode = Mode.MANUAL
                shared.manual_command = Manual.RIGHT
                shared.release()

        elif cmd == "F":
            # a manual action implicitly switches to manual mode
            if shared.acquire():
                shared.mode = Mode.MANUAL
                shared.manual_command = Manual.STRAIGHT
                shared.release()

        elif cmd == "Z":
            # second prompt for the zone rectangle
            coords = input("Enter zone Xmin Xmax Ymin Ymax: ").strip().split()
            try:
                xmin = int(coords[0])
                xmax = int(coords[1])
                ymin = int(coords[2])
                ymax = int(coords[3])
            except (ValueError, IndexError):
                print("Invalid zone buddy.")
                continue
            if shared.acquire():
                shared.mode = Mode.ZONE
                shared.zone_xmin = xmin
                shared.zone_xmax = xmax
                shared.zone_ymin = ymin
                shared.zone_ymax = ymax
                shared.release()

        elif cmd == "P":
            # reset position and heading, keep the map
            coords = input("Enter pose X Y H (H 0-7): ").strip().split()
            try:
                xp = int(coords[0])
                yp = int(coords[1])
                hp = int(coords[2])
            except (ValueError, IndexError):
                print("Invalid pose buddy.")
                continue
            if hp < 0 or hp > 7:
                print("Heading must be 0-7 buddy.")
                continue
            if shared.acquire():
                shared.pose = (xp, yp, hp)
                shared.release()

        elif cmd == "V":
            # second prompt for the filename
            filename = input("Filename: ").strip()
            if not filename:
                print("No filename, skipped.")
                continue
            if shared.acquire():
                shared.save_filename = filename
                shared.release()

        elif cmd == "C":
            if shared.acquire():
                shared.clear = True
                shared.release()

        elif cmd == "Q":
            if shared.acquire():
                shared.quit = True
                shared.release()
            return

        else:
            print("Invalid input bro.")

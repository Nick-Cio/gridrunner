import time
import math
import threading
from enum import Enum

from robot.utils import calc_move, calc_turn, calc_uturn
from robot.config import DEADEND_BRAKE_PAUSE_S, BLOCKAGE_TTL_S, TURN_SETTLE_S
from robot.config import STUCK_TURN_LIMIT, MIDSTREET_UTURN_LIMIT, MIDSTREET_UTURN_PAUSE_S
from robot.config import UNEXPLORED_TRAVERSE_COST, UNKNOWN_TRAVERSE_COST, CARTESIAN_PENALTY
from robot.ui import runui, Mode, Manual
from world.map import Map, load_map
from world.intersections import Status


# ============================================================
# Action enum
# ============================================================

class Action(Enum):
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    STRAIGHT = "straight"
    IDLE = "idle"
    DONE = "done"


# ============================================================
# Map update helpers
# ============================================================

def mark_first_intersection(map, x, y, heading, u_turned, road_ahead):
    """Populate streets at the starting intersection from compound_start info."""
    if u_turned:
        map.set_street(x, y, calc_uturn(heading), Status.DEADEND)
    else:
        map.set_street(x, y, calc_uturn(heading), Status.UNEXPLORED)

    map.set_street(x, y, (heading + 3) % 8, Status.NONEXISTENT)
    map.set_street(x, y, (heading - 3) % 8, Status.NONEXISTENT)

    if road_ahead:
        map.set_street(x, y, heading, Status.UNEXPLORED)
        map.set_street(x, y, (heading + 1) % 8, Status.NONEXISTENT)
        map.set_street(x, y, (heading - 1) % 8, Status.NONEXISTENT)
    else:
        map.set_street(x, y, heading, Status.NONEXISTENT)


def mark_arrived_intersection(map, x, y, heading, road_ahead):
    """Populate streets after entering a new intersection along heading."""
    map.set_street(x, y, (heading + 3) % 8, Status.NONEXISTENT)
    map.set_street(x, y, (heading - 3) % 8, Status.NONEXISTENT)

    if road_ahead:
        # only promote if untouched; preserves CONNECTED from prior visits
        if map.intersections[(x, y)].streets[heading] == Status.UNKNOWN:
            map.set_street(x, y, heading, Status.UNEXPLORED)
        map.set_street(x, y, (heading + 1) % 8, Status.NONEXISTENT)
        map.set_street(x, y, (heading - 1) % 8, Status.NONEXISTENT)
    else:
        map.set_street(x, y, heading, Status.NONEXISTENT)


def apply_turn_to_map(map, x, y, heading_before, turncount, direction):
    """update map streets after a turn and return the new heading.
    direction: +1 right (turncount negative), -1 left (turncount positive).
    """
    diff = abs(turncount) % 8

    # Full 360+: every slot except the heading is sweep-empty.
    if diff == 0 and abs(turncount) >= 8:
        for i in range(8):
            if i != heading_before:
                map.set_street(x, y, i, Status.NONEXISTENT)
    # Partial sweep > 45 deg: intermediate slots are sweep-empty.
    elif diff > 1:
        if direction == 1:
            start = heading_before - diff
            for i in range(start + 1, heading_before):
                map.set_street(x, y, i % 8, Status.NONEXISTENT)
        else:
            for j in range(1, diff):
                map.set_street(x, y, (heading_before + j) % 8, Status.NONEXISTENT)

    new_heading = calc_turn(heading_before, turncount)

    if map.intersections[(x, y)].streets[new_heading] == Status.UNKNOWN:
        map.set_street(x, y, new_heading, Status.UNEXPLORED)

    # We've confirmed a street at new_heading: +/-45 deg neighbors cannot exist
    if map.intersections[(x, y)].streets[new_heading] != Status.NONEXISTENT:
        for nbr in ((new_heading + 1) % 8, (new_heading - 1) % 8):
            if map.intersections[(x, y)].streets[nbr] == Status.UNKNOWN:
                map.set_street(x, y, nbr, Status.NONEXISTENT)

    return new_heading


# ============================================================
# Action helpers (motor-side)
# ============================================================

def update_blockage_ahead(behaviors, map, x, y, heading, front_only=False):
    # Reads ultrasounds and marks the front and (if called) the side streets as well
    # Skip sides after a pullthrough since the side sensors no longer aim at them

    blockages = behaviors.check_blockages()
    left_b, front_b, right_b = blockages

    l, m, r = behaviors.proximity.left, behaviors.proximity.middle, behaviors.proximity.right
    # print(f"[buckets] L d={l.read():.2f} (20:{l.samples20} 65:{l.samples65} 92:{l.samples92})  "
    #       f"M d={m.read():.2f} (20:{m.samples20} 65:{m.samples65} 92:{m.samples92})  "
    #       f"R d={r.read():.2f} (20:{r.samples20} 65:{r.samples65} 92:{r.samples92})")

    if heading % 2 == 0:
        bucket = 1   # below65cm
    else:
        bucket = 2   # below92cm

    map.set_blocked(x, y, heading, front_b[bucket])
    if not front_only:
        map.set_blocked(x, y, (heading + 2) % 8, left_b[bucket])
        map.set_blocked(x, y, (heading - 2) % 8, right_b[bucket])


def snap_to_existing_road(intersection, gyro_heading, heading_before, direction, turncount):
    # trust the gyro unless it points at a slot we know is empty
    if intersection.streets[gyro_heading] != Status.NONEXISTENT:
        return gyro_heading

    # real road is one slot off. Big turns (4+ slots) tend to overshoot, so
    # check the reverse (opposite) side first; smaller turns check the turn side first.
    turn_sign = -direction
    if abs(turncount) >= 4:
        offsets = (-1, 1)
    else:
        offsets = (1, -1)
    for offset in offsets:
        h = (gyro_heading + offset * turn_sign) % 8
        if h == heading_before:
            continue
        if intersection.streets[h] != Status.NONEXISTENT:
            return h
    return gyro_heading


def turn_step(behaviors, map, x, y, heading, direction, shared):
    """Execute one turn and update the map. Returns new heading.
    """
    angle_turned = behaviors.turn(direction)
    if angle_turned is None:
        # turn timed out, treat as no movement and keep our heading
        return heading
    turncount = int(angle_turned / 45)

    intersection = map.intersections[(x, y)]

    # Gyro guess, then map-correct it against what we knew before this turn.
    gyro_heading = calc_turn(heading, turncount)
    new_heading = snap_to_existing_road(intersection, gyro_heading, heading, direction, turncount)

    # gyro guessed a road the map says isn't there: weird turn, show it and where snap landed
    if intersection.streets[gyro_heading] == Status.NONEXISTENT:
        print(f"weird turn: gyro guessed h={gyro_heading}, no road there, measured {angle_turned} deg")
        print(f"snapped to h={new_heading}")

    if new_heading != gyro_heading:
        # Recount slots so the map updates match where we actually stopped.
        slots = ((new_heading - heading) * -direction) % 8
        turncount = -direction * slots

    apply_turn_to_map(map, x, y, heading, turncount, direction)

    # If even the snap found no real street, we're genuinely lost: stop and say so.
    if intersection.streets[new_heading] == Status.NONEXISTENT:
        print(f"ROBOT LOST: no road within 45 deg of h={new_heading} (measured {angle_turned} deg). Braking, switching to manual.")
        behaviors.drive.brake()
        if shared.acquire():
            shared.mode = Mode.MANUAL
            shared.release()

    # After a turn, rest so the ultrasound buckets fill at the new heading
    behaviors.drive.brake()
    time.sleep(TURN_SETTLE_S)
    update_blockage_ahead(behaviors, map, x, y, new_heading, front_only=True)

    return new_heading


def smart_street_follow(behaviors, heading, shared):
    """Drive one segment with mid-street obstacle detection.
    Returns: advanced, uturn after deadend, uturn after blockage, stopped
    """

    original_heading = heading
    prev_result = None
    # count blockage u-turns so we stop bouncing after a few
    blocked_uturns = 0

    while True:
        result = behaviors.follow_line()

        if result == "FOUND_INTERSECTION":
            # came back from a dead-end u-turn
            if prev_result == "FOUND_END":
                return "UTURN_AFTER_DEADEND", heading
            # arrived facing original direction: advanced through
            if heading == original_heading:
                return "ADVANCED", heading
            # ended up facing reverse: u-turn'd from blockage and made it back
            return "UTURN_AFTER_BLOCKAGE", heading

        if result == "FOUND_END":
            # First-time dead-end: u-turn and try to drive back to the prev intersection
            behaviors.turn(-1)
            behaviors.drive.brake()
            time.sleep(DEADEND_BRAKE_PAUSE_S)
            heading = calc_uturn(heading)
            prev_result = "FOUND_END"
            continue

        if result == "FOUND_BLOCKAGE":
            # if after dead end wait 4eva
            if prev_result == "FOUND_END":
                while behaviors.proximity.middle.blocked_below20cm():
                    behaviors.drive.brake()
                    if shared.stop or shared.quit:
                        return "STOPPED", heading
                    time.sleep(0.05)
                continue

            # bounced too many times: face the way we came in and wait it out
            if blocked_uturns >= MIDSTREET_UTURN_LIMIT:
                if heading != calc_uturn(original_heading):
                    behaviors.turn(-1)
                    heading = calc_uturn(heading)
                while behaviors.proximity.middle.blocked_below20cm():
                    behaviors.drive.brake()
                    if shared.stop or shared.quit:
                        return "STOPPED", heading
                    time.sleep(0.05)
                continue

            # Give the obstacle a moment to clear, scanning instead of just sleeping.
            behaviors.drive.brake()
            waited = 0.0
            while behaviors.proximity.middle.blocked_below20cm() and waited < MIDSTREET_UTURN_PAUSE_S:
                time.sleep(0.05)
                waited += 0.05

            # Still blocked: u-turn and try the other way.
            if behaviors.proximity.middle.blocked_below20cm():
                behaviors.turn(-1)
                heading = calc_uturn(heading)
                blocked_uturns += 1
                prev_result = "FOUND_BLOCKAGE"
            continue


def drive_segment(behaviors, map, x, y, heading, last_intersection, shared):
    """Follow one segment with smart obstacle handling
    """
    original_heading = heading
    result, heading = smart_street_follow(behaviors, heading, shared)

    if result == "STOPPED":
        # stop/quit hit mid-street
        return x, y, heading, last_intersection, None

    if result == "ADVANCED":
        prev = last_intersection
        x, y = calc_move(x, y, original_heading)
        last_intersection = map.getintersection(x, y)
        map.set_street(prev.x, prev.y, original_heading, Status.CONNECTED)
        # Sides are aligned with the cardinal side streets
        update_blockage_ahead(behaviors, map, x, y, heading)
        road_ahead = behaviors.pullthrough()
        mark_arrived_intersection(map, x, y, heading, road_ahead)
        return x, y, heading, last_intersection, prev

    if result == "UTURN_AFTER_DEADEND":
        # back at start, facing reverse. Mark the forward street DEADEND.
        map.set_street(last_intersection.x, last_intersection.y, original_heading, Status.DEADEND)
        update_blockage_ahead(behaviors, map, x, y, heading)
        behaviors.pullthrough()
        return x, y, heading, last_intersection, None

    # UTURN_AFTER_BLOCKAGE
    update_blockage_ahead(behaviors, map, x, y, heading)
    behaviors.pullthrough()
    return x, y, heading, last_intersection, None


# ============================================================
# Navigation helpers
# ============================================================

def shortest_turn_dir(heading, target_h):
    """Mod-8 shortest spin: +1 right, -1 left, 0 if aligned (180 ties go right)."""
    delta = (target_h - heading) % 8
    if delta == 0:
        return 0
    if delta < 4:
        return -1
    return 1


def pick_nearest_unexplored_heading(intersection, heading):
    """Heading of the UNEXPLORED-and-unblocked street nearest (mod 8) to heading."""
    if intersection.streets[heading] == Status.UNEXPLORED and not intersection.blocked[heading]:
        return heading
    best = None
    best_dist = 100
    for h in range(8):
        if intersection.streets[h] == Status.UNEXPLORED and not intersection.blocked[h]:
            delta = (h - heading) % 8
            d = min(delta, 8 - delta)
            if d < best_dist:
                best_dist = d
                best = h
    return best


def nearest_unknown_direction(intersection, heading):
    """Direction (+1 right, -1 left) toward the nearest UNKNOWN slot."""
    for d in range(1, 5):
        right_unknown = intersection.streets[(heading - d) % 8] == Status.UNKNOWN
        left_unknown  = intersection.streets[(heading + d) % 8] == Status.UNKNOWN
        if right_unknown and not left_unknown:
            return 1
        if left_unknown and not right_unknown:
            return -1
        if right_unknown and left_unknown:
            return 1


def build_unfinished(map):
    """Set of (x,y) for intersections with at least one street worth exploring.
    Worth it = UNEXPLORED (saw the line) or UNKNOWN (never scanned), and not blocked.
    """
    unfinished = set()
    for coord in map.intersections:
        inter = map.intersections[coord]
        for h in range(8):
            if inter.blocked[h]:
                continue
            if inter.streets[h] == Status.UNEXPLORED or inter.streets[h] == Status.UNKNOWN:
                unfinished.add(coord)
                break
    return unfinished


def has_unexplored(map):
    """tell a truly complete map from one we're just trapped in
    """
    for inter in map.intersections.values():
        for h in range(8):
            if inter.streets[h] == Status.UNEXPLORED:
                return True
    return False


def is_drivable(intersection, heading):
    """True if we can actually drive straight in this direction.
    Drivable = UNEXPLORED or CONNECTED (and cnt be blocked)
    """
    status = intersection.streets[heading]
    if status not in (Status.UNEXPLORED, Status.CONNECTED):
        return False
    if intersection.blocked[heading]:
        return False
    return True


def in_zone(x, y, xmin, xmax, ymin, ymax):
    """True if (x, y) inside the zone (e)rect"""
    if x < xmin or x > xmax:
        return False
    if y < ymin or y > ymax:
        return False
    return True


# ============================================================
# Per-mode action pickers (given state return action)
# ============================================================

def act_manual(manual_cmd):
    """Map directly to action."""
    if manual_cmd == Manual.LEFT:
        return Action.TURN_LEFT
    if manual_cmd == Manual.RIGHT:
        return Action.TURN_RIGHT
    if manual_cmd == Manual.STRAIGHT:
        return Action.STRAIGHT
    return Action.IDLE


def act_goto(x, y, xgoal, ygoal, last_intersection, heading):
    """One step toward goal.
    """
    if (x, y) == (xgoal, ygoal):
        return Action.DONE

    target_h = last_intersection.next_heading
    if target_h is None:
        # No path right now.
        return Action.IDLE

    if heading == target_h:
        return Action.STRAIGHT

    direction = shortest_turn_dir(heading, target_h)
    if direction == 1:
        return Action.TURN_RIGHT
    return Action.TURN_LEFT


def act_explore(map, last_intersection, x, y, heading):
    """One step of exploration. Three phases checked in order:
    1) any UNKNOWN at this intersection -> turn to scan
    2) any drivable UNEXPLORED here -> face it / drive it
    3) otherwise dijkstra to the nearest unfinished intersection
    """
    # 1: scan UNKNOWN slots by turning toward the nearest one.
    if Status.UNKNOWN in last_intersection.streets:
        direction = nearest_unknown_direction(last_intersection, heading)
        if direction == 1:
            return Action.TURN_RIGHT
        return Action.TURN_LEFT

    # 2: drivable UNEXPLORED at this intersection.
    target_h = pick_nearest_unexplored_heading(last_intersection, heading)
    if target_h is not None:
        if heading == target_h:
            return Action.STRAIGHT
        direction = shortest_turn_dir(heading, target_h)
        if direction == 1:
            return Action.TURN_RIGHT
        return Action.TURN_LEFT

    # 3: nothing local. Find the nearest unfinished intersection.
    unfinished = build_unfinished(map)
    if not unfinished:
        return Action.DONE

    # cost-fill from current position to find the nearest unfinished
    map.dijkstra(x, y)
    nearest = None
    best_cost = math.inf
    for c in unfinished:
        cost = map.intersections[c].cost
        if cost < best_cost:
            best_cost = cost
            nearest = c
    if nearest is None or best_cost == math.inf:
        return Action.DONE

    # now plan toward that target so next_heading points the right way
    map.dijkstra(nearest[0], nearest[1])
    target_h = last_intersection.next_heading
    if target_h is None:
        return Action.DONE

    if heading == target_h:
        return Action.STRAIGHT
    direction = shortest_turn_dir(heading, target_h)
    if direction == 1:
        return Action.TURN_RIGHT
    return Action.TURN_LEFT


def pick_unexplored_in_zone(intersection, x, y, heading, zone):
    """Heading of the nearest UNEXPLORED+unblocked street whose far end stays in the zone."""
    xmin, xmax, ymin, ymax = zone
    best = None
    best_d = 100
    for h in range(8):
        if intersection.streets[h] != Status.UNEXPLORED or intersection.blocked[h]:
            continue
        nx, ny = calc_move(x, y, h)
        if not in_zone(nx, ny, xmin, xmax, ymin, ymax):
            continue   # never drive a street that leaves the zone
        d = min((h - heading) % 8, (heading - h) % 8)
        if d < best_d:
            best_d = d
            best = h
    return best


def act_zone_explore(map, last_intersection, x, y, heading, zone):
    """Explore inside the zone, never driving a street that leaves it.
    Returns DONE when there's nothing left to explore within the zone.
    """
    xmin, xmax, ymin, ymax = zone

    # 1: scan UNKNOWN slots in place (turning never leaves the intersection)
    if Status.UNKNOWN in last_intersection.streets:
        direction = nearest_unknown_direction(last_intersection, heading)
        return Action.TURN_RIGHT if direction == 1 else Action.TURN_LEFT

    # 2: drive an UNEXPLORED street whose far end stays in the zone
    target_h = pick_unexplored_in_zone(last_intersection, x, y, heading, zone)

    # 3: nothing here -> route (staying in-zone) to the nearest in-zone unfinished
    if target_h is None:
        unfinished = [c for c in build_unfinished(map)
                      if in_zone(c[0], c[1], xmin, xmax, ymin, ymax)]
        if not unfinished:
            return Action.DONE   # zone fully explored

        map.dijkstra(x, y, zone=zone)   # cost from us, never leaving the zone
        nearest, best = None, math.inf
        for c in unfinished:
            if map.intersections[c].cost < best:
                best, nearest = map.intersections[c].cost, c
        if nearest is None or best == math.inf:
            return Action.DONE

        map.dijkstra(nearest[0], nearest[1], zone=zone)   # tree toward it, in-zone
        target_h = last_intersection.next_heading
        if target_h is None:
            return Action.DONE

    if heading == target_h:
        return Action.STRAIGHT
    direction = shortest_turn_dir(heading, target_h)
    return Action.TURN_RIGHT if direction == 1 else Action.TURN_LEFT


# ============================================================
# Directed exploration: wehn no path exist
# ============================================================

def pick_unexplored_toward_goal(intersection, x, y, xgoal, ygoal):
    """Heading of the UNEXPLORED-and-unblocked street that gets us closest to
    the goal after one segment of driving.
    """
    best = None
    best_dist = math.inf
    for h in range(8):
        if intersection.streets[h] != Status.UNEXPLORED:
            continue
        if intersection.blocked[h]:
            continue
        nx, ny = calc_move(x, y, h)
        dist = math.sqrt((xgoal - nx) ** 2 + (ygoal - ny) ** 2)
        if dist < best_dist:
            best_dist = dist
            best = h
    return best


def act_directed(map, last_intersection, x, y, heading, xgoal, ygoal):
    """Drive toward the frontier that best balances dist to goal and dist to curr basically A star)
    """
    if Status.UNKNOWN in last_intersection.streets:
        direction = nearest_unknown_direction(last_intersection, heading)
        if direction == 1:
            return Action.TURN_RIGHT
        return Action.TURN_LEFT

    unfinished = build_unfinished(map)
    if not unfinished:
        return Action.DONE

    # distance from us to every node, used as the "cost to get there" (red)
    map.dijkstra(x, y)

    seeds = {}
    scores = {}
    chosen = None
    best_total = math.inf
    for (ux, uy) in unfinished:
        inter = map.intersections[(ux, uy)]
        # pick the street worth exploring with the cheapest traverse + dist to goal.
        # unexplored streets cost less than unknown ones (we at least saw the line).
        blue = math.inf
        green = math.inf
        for h in range(8):
            if inter.blocked[h]:
                continue
            if inter.streets[h] == Status.UNEXPLORED:
                traverse = UNEXPLORED_TRAVERSE_COST
            elif inter.streets[h] == Status.UNKNOWN:
                traverse = UNKNOWN_TRAVERSE_COST
            else:
                continue
            nx, ny = calc_move(ux, uy, h)
            d = math.sqrt((xgoal - nx) ** 2 + (ygoal - ny) ** 2)
            dist_cost = CARTESIAN_PENALTY * d
            if traverse + dist_cost < blue + green:
                blue = traverse
                green = dist_cost

        if blue == math.inf:
            continue

        seeds[(ux, uy)] = blue + green

        # full score for display: cost to get there + traverse + dist to goal
        red = inter.cost
        if red != math.inf:
            total = red + blue + green
            scores[(ux, uy)] = (total, red, blue, green)
            if total < best_total:
                best_total = total
                chosen = (ux, uy)

    map.frontier_scores = scores
    map.chosen_frontier = chosen

    map.dijkstra_multi(seeds)
    map.changed = True

    if last_intersection.next_heading is not None:
        target_h = last_intersection.next_heading
    else:
        target_h = pick_unexplored_toward_goal(last_intersection, x, y, xgoal, ygoal)
        if target_h is None:
            return Action.DONE

    if heading == target_h:
        return Action.STRAIGHT
    direction = shortest_turn_dir(heading, target_h)
    if direction == 1:
        return Action.TURN_RIGHT
    return Action.TURN_LEFT


# ============================================================
# Action executor
# ============================================================

def execute(action, behaviors, map, x, y, heading, last_intersection, shared):
    """Run the chosen action and return the new (x, y, heading, last_intersection)."""
    if action == Action.TURN_LEFT:
        new_h = turn_step(behaviors, map, x, y, heading, -1, shared)
        return x, y, new_h, last_intersection

    if action == Action.TURN_RIGHT:
        new_h = turn_step(behaviors, map, x, y, heading, 1, shared)
        return x, y, new_h, last_intersection

    if action == Action.STRAIGHT:
        nx, ny, nh, nli, prev = drive_segment(behaviors, map, x, y, heading, last_intersection, shared)
        return nx, ny, nh, nli

    # IDLE
    behaviors.drive.brake()
    time.sleep(0.05)
    return x, y, heading, last_intersection


# ============================================================
# Prompts
# ============================================================

def prompt_pose():
    while True:
        try:
            parts = input("Enter starting pose (x y h) must be 0-7: ").strip().split()
            x, y, h = int(parts[0]), int(parts[1]), int(parts[2])
            if 0 <= h <= 7:
                return x, y, h
        except (ValueError, IndexError):
            pass
        print("Invalid input buddy.")


# ============================================================
# Startup
# ============================================================

def startup(behaviors):
    """Drive to first intersection, init/load map, ask pose, mark first intersection."""
    u_turned, road_ahead = behaviors.compound_start()

    while True:
        choice = input("Map: (B)lank or (L)oad from file? ").strip().upper()
        if choice in ('B', 'L'):
            break
        print("Enter B or L.")

    if choice == 'L':
        filename = input("Filename to load: ").strip()
        map = load_map(filename)
        map.show()
    else:
        map = Map()

    x, y, heading = prompt_pose()
    last_intersection = map.getintersection(x, y)

    # For blank maps, derive starting-intersection streets from compound_start.
    # Loaded maps already carry that information.
    if choice == 'B':
        mark_first_intersection(map, x, y, heading, u_turned, road_ahead)

    return map, x, y, heading, last_intersection


# ============================================================
# Brain
# ============================================================

# Single loop for all modes:
#   1. read sensors
#   2. handle global flags like quit/stop
#   3. pick one action
#   4. check if blocked (change to ilde)
#   5. execute the action
#   6. refresh front blockage

def brain(behaviors, shared):
    map, x, y, heading, last_intersection = startup(behaviors)

    # Startup is done, spawn UI thread
    uithread = threading.Thread(name="UIThread", target=runui, args=(shared,))
    uithread.daemon = True
    uithread.start()

    # NFC tag for the intersection we are currently on
    intersection_id = behaviors.nfc.read()
    if intersection_id is not None:
        last_intersection.nfc_id = intersection_id

    # Track what Dijkstra was last run for and what mode we came from,
    # so we know when GOTO needs a fresh plan.
    current_goal = None
    prev_mode = Mode.MANUAL

    # stuck guard: count turns with no forward progress, remember where we came from
    turns_without_progress = 0
    came_from = calc_uturn(heading)
    prev_blocked_count = 0
    self_cleared = False   # True when WE wiped blockages because trapped (vs obstacle moving)
    prev_unknown_count = 0   # UNKNOWN streets last loop; a drop means we learned something
    stuck_announced = False  # so the "max turns" message prints once per stuck episode

    while True:
        # Latch any new NFC tag the worker thread has seen since last loop
        new_id = behaviors.nfc.read()
        if new_id is not None:
            intersection_id = new_id

            # Check if this tag is already stored at some other coordinate.
            tag_owner = None
            for coord, inter in map.intersections.items():
                if inter.nfc_id == new_id and coord != (x, y):
                    tag_owner = coord
                    break

            if tag_owner is not None:
                # Tag belongs to a different spot -> we're lost.
                behaviors.drive.brake()
                print("NFC tag belongs to %s but we think we're at (%d, %d): lost"
                      % (tag_owner, x, y))
                if shared.acquire():
                    shared.mode = Mode.MANUAL
                    shared.release()
            elif last_intersection.nfc_id is None:
                # First time here: learn this intersection's tag.
                last_intersection.nfc_id = new_id
            elif last_intersection.nfc_id != new_id:
                # We have a tag for here and it doesn't match -> we're lost.
                behaviors.drive.brake()
                print("intersection coordinate (%d, %d) does not match NFC tag reading"
                      % (x, y))
                if shared.acquire():
                    shared.mode = Mode.MANUAL
                    shared.release()

        # Forget stale blockages so a moved obstacle stops blocking a route
        map.expire_blockages(BLOCKAGE_TTL_S)

        # if a blockage cleared give another try -- but only when an obstacle
        # moved on its own. If WE wiped blockages because we were trapped, don't
        # reset, or the stuck guard could never trip in a boxed-in corner.
        blocked_count = 0
        for inter in map.intersections.values():
            blocked_count += sum(inter.blocked)
        if blocked_count < prev_blocked_count and not self_cleared:
            turns_without_progress = 0
        prev_blocked_count = blocked_count
        self_cleared = False

        # Learning new map info (resolving an UNKNOWN street) is also progress, even
        # without moving -- so scanning turns at a real intersection don't count as
        # stuck. A trapped spot stops resolving anything new, so its counter still climbs.
        unknown_count = 0
        for inter in map.intersections.values():
            for h in range(8):
                if inter.streets[h] == Status.UNKNOWN:
                    unknown_count += 1
        if unknown_count < prev_unknown_count:
            turns_without_progress = 0
        prev_unknown_count = unknown_count

        # ----- interct w/ shared data: give position, look at states, update one shots -----
        if shared.acquire():
            quit_flag = shared.quit
            stop_flag = shared.stop
            mode = shared.mode
            xgoal = shared.xgoal
            ygoal = shared.ygoal
            zone_xmin = shared.zone_xmin
            zone_xmax = shared.zone_xmax
            zone_ymin = shared.zone_ymin
            zone_ymax = shared.zone_ymax
            manual_cmd = shared.manual_command
            save_filename = shared.save_filename
            clear_flag = shared.clear
            pose_cmd = shared.pose

            # If stop was requested force mode back to manual and drop any cmd
            if stop_flag:
                shared.stop = False
                shared.mode = Mode.MANUAL
                shared.manual_command = None
                mode = Mode.MANUAL
                manual_cmd = None

            if manual_cmd is not None:
                shared.manual_command = None
            if save_filename:
                shared.save_filename = ""
            if clear_flag:
                shared.clear = False
            if pose_cmd is not None:
                shared.pose = None

            # Publish current pose so the ROS thread can broadcast it
            shared.robotx = x
            shared.roboty = y
            shared.robotheading = heading
            shared.intersectionid = intersection_id

            shared.release()

        # ----- global flags -----
        if quit_flag:
            return

        if stop_flag:
            behaviors.drive.brake()
            map.clear_dijkstra()
            current_goal = None
            print("Stopped: manual mode")

        if save_filename:
            map.save(save_filename)

        if clear_flag:
            map.clear_blockages()
            print("Cleared all blockages.")

        if pose_cmd is not None:
            x, y, heading = pose_cmd
            last_intersection = map.getintersection(x, y)
            # new position, so force a fresh plan next loop
            map.changed = True
            print("Pose reset to (%d, %d) heading %d" % (x, y, heading))

        # ----- pick the next action based on mode -----
        if mode == Mode.MANUAL:
            action = act_manual(manual_cmd)

        elif mode == Mode.GOTO:
            if xgoal is None or ygoal is None:
                action = Action.IDLE
            else:
                # Re-run dijkstra only when something might have changed the plan:
                #   - map changed (new CONNECTED, blockage flipped, blockages cleared)
                #   - different goal than last time
                #   - just entered GOTO from another mode
                need_replan = (map.changed or current_goal != (xgoal, ygoal) or prev_mode != Mode.GOTO)
                if need_replan:
                    map.dijkstra(xgoal, ygoal)
                    map.changed = False
                    current_goal = (xgoal, ygoal)

                if last_intersection.cost == math.inf:
                    action = act_directed(map, last_intersection, x, y, heading, xgoal, ygoal)
                    if action == Action.DONE:
                        if map.has_blockages():
                            print("Explore exhausted; clearing blockages to re-attempt.")
                            map.clear_blockages()
                            map.changed = True
                            self_cleared = True
                        else:
                            print("Goal unreachable; all paths explored and unblocked.")
                            if shared.acquire():
                                shared.mode = Mode.MANUAL
                                shared.release()
                        action = Action.IDLE
                else:
                    # real path found, no need to show explore frontiers
                    map.frontier_scores = {}
                    map.chosen_frontier = None
                    action = act_goto(x, y, xgoal, ygoal, last_intersection, heading)

        elif mode == Mode.EXPLORE:
            action = act_explore(map, last_intersection, x, y, heading)
            # If unexplored streets still exist behind blockages we're trapped
            # clear blockages
            if action == Action.DONE and has_unexplored(map) and map.has_blockages():
                print("Explore trapped; clearing blockages to re-attempt.")
                map.clear_blockages()
                map.changed = True
                self_cleared = True
                action = Action.IDLE

        elif mode == Mode.ZONE:
            if zone_xmin is None or zone_xmax is None:
                action = Action.IDLE
            elif in_zone(x, y, zone_xmin, zone_xmax, zone_ymin, zone_ymax):
                # Inside the zone: keep exploring within it, never leaving. When
                # there's nothing left to explore in-zone, hold position (remain
                # in the zone until a new zone command takes over).
                map.zone = (zone_xmin, zone_xmax, zone_ymin, zone_ymax)
                map.frontier_scores = {}
                map.chosen_frontier = None
                action = act_zone_explore(map, last_intersection, x, y, heading,
                                          (zone_xmin, zone_xmax, zone_ymin, zone_ymax))
                if action == Action.DONE:
                    action = Action.IDLE
            else:
                # show the zone on the map and head for its middle, same logic as a goal
                map.zone = (zone_xmin, zone_xmax, zone_ymin, zone_ymax)
                xgoal = round((zone_xmin + zone_xmax) / 2)
                ygoal = round((zone_ymin + zone_ymax) / 2)
                # only replan when the map, zone center, or mode changed
                need_replan = (map.changed or current_goal != (xgoal, ygoal) or prev_mode != Mode.ZONE)
                if need_replan:
                    map.dijkstra(xgoal, ygoal)
                    map.changed = False
                    current_goal = (xgoal, ygoal)

                if last_intersection.cost == math.inf:
                    action = act_directed(map, last_intersection, x, y, heading, xgoal, ygoal)
                    if action == Action.DONE:
                        if map.has_blockages():
                            print("Zone path blocked; clearing blockages to re-attempt.")
                            map.clear_blockages()
                            map.changed = True
                            self_cleared = True
                        else:
                            print("Zone unreachable; all paths explored and unblocked.")
                            if shared.acquire():
                                shared.mode = Mode.MANUAL
                                shared.release()
                        action = Action.IDLE
                else:
                    # real path found, no need to show explore frontiers
                    map.frontier_scores = {}
                    map.chosen_frontier = None
                    action = act_goto(x, y, xgoal, ygoal, last_intersection, heading)

        else:
            action = Action.IDLE

        # ----- stuck guard: too many turns without moving forward so go back where we cam from -----
        if (mode != Mode.MANUAL and action != Action.DONE and turns_without_progress > STUCK_TURN_LIMIT and came_from is not None):
            if heading != came_from:
                direction = shortest_turn_dir(heading, came_from)
                if direction == 1:
                    action = Action.TURN_RIGHT
                else:
                    action = Action.TURN_LEFT
            else:
                # Faced the way back and still stuck: stay in this mode and hold
                # position, waiting indefinitely for the blockage to clear. When it
                # does, a reset above releases the guard and we resume automatically.
                if not stuck_announced:
                    print("Stuck: max turns reached. Holding position, waiting for the path to clear...")
                    stuck_announced = True
                action = Action.IDLE
        else:
            stuck_announced = False

        # ----- check bloackage -----
        if action == Action.STRAIGHT and not is_drivable(last_intersection, heading):
            fwd = last_intersection.streets[heading]
            if last_intersection.blocked[heading]:
                print("Nah bro: forward street is blocked")
            else:
                print(f"Nah bro: forward street is {fwd.value}")
            action = Action.IDLE

        # ----- mode-complete checker -----
        if action == Action.DONE:
            if mode == Mode.GOTO:
                print("Arrived at goal: sigma status confirmed")
            elif mode == Mode.EXPLORE:
                print("Map complete: sigma status confirmed")
            elif mode == Mode.ZONE:
                print("Entered safe zone: sigma status confirmed")
            map.clear_dijkstra()
            current_goal = None
            if shared.acquire():
                shared.mode = Mode.MANUAL
                shared.release()
            action = Action.IDLE

        # ----- draw now, with the chosen frontier, before we move -----
        map.show_with_robot(x, y, heading)

        # ----- actually execute the action -----
        prev_x, prev_y = x, y
        x, y, heading, last_intersection = execute(action, behaviors, map, x, y, heading, last_intersection, shared)

        # ----- track turns with no forward progress -----
        if mode == Mode.MANUAL:
            turns_without_progress = 0
        elif (x, y) != (prev_x, prev_y):
            # moved to a new intersection, remember the way back
            turns_without_progress = 0
            came_from = calc_uturn(heading)
        elif action == Action.TURN_LEFT or action == Action.TURN_RIGHT:
            turns_without_progress += 1

        # ----- refresh front blockage so the next iteration ---
        update_blockage_ahead(behaviors, map, x, y, heading, front_only=True)

        prev_mode = mode
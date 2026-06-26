from world.intersections import Intersection, Status
from robot.utils import calc_move, calc_uturn
import matplotlib.pyplot as plt
import math
import pickle
import bisect
import time


# promotion tiers: writes only allowed when new tier > current tier
STATUS_TIER = {
    Status.UNKNOWN: 0,
    Status.UNEXPLORED: 1,
    Status.NONEXISTENT: 1,
    Status.CONNECTED: 2,
    Status.DEADEND: 2,
}


class Map():

    def __init__(self):
        self.intersections = {}
        # flag: set when something Dijkstra cares about changes.
        self.changed = False
        # (x, y, h) -> time.time() when that street was last marked blocked.
        self.block_times = {}
        # active safe zone (xmin, xmax, ymin, ymax) to draw, or None.
        self.zone = None
        # directed-explore frontier scores to draw: (x,y) -> (total, red, blue, green)
        self.frontier_scores = {}
        # (x,y) of the frontier directed-explore picked, or None
        self.chosen_frontier = None


    def getintersection(self,x,y):
        if (x,y) not in self.intersections:
            self.intersections[(x,y)] = Intersection(x,y)
        return self.intersections[(x,y)]


    def set_street(self, x, y, heading, state):
        inter = self.getintersection(x, y)
        if STATUS_TIER[state] <= STATUS_TIER[inter.streets[heading]]:
            return

        inter.streets[heading] = state

        if state == Status.CONNECTED:
            self.changed = True
            nx, ny = calc_move(x, y, heading)
            neighbor = self.getintersection(nx, ny)
            opp = calc_uturn(heading)
            if STATUS_TIER[Status.CONNECTED] > STATUS_TIER[neighbor.streets[opp]]:
                neighbor.streets[opp] = Status.CONNECTED


    # draws just the map no robot
    def draw_base(self):
        street_color = {
            Status.UNKNOWN : "black",
            Status.NONEXISTENT : "lightgray",
            Status.UNEXPLORED : "blue",
            Status.DEADEND : "red",
            Status.CONNECTED : "green"
            }
        street_length = {
            0 : (0, 0.5),
            1 : (-0.5, 0.5),
            2 : (-0.5, 0),
            3 : (-0.5, -0.5),
            4 : (0, -0.5),
            5 : (0.5, -0.5),
            6 : (0.5, 0),
            7 : (0.5, 0.5)
        }

        # Clear the current, or create a new figure.
        plt.clf()
        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.gca().set_xlim(-8.5, 8.5)
        plt.gca().set_ylim(-5.5, 5.5)
        plt.gca().set_aspect('equal')

        # Show all the possible locations (one batched plot call, not 187).
        gx = [x for x in range(-8, 9) for _ in range(-5, 6)]
        gy = [y for _ in range(-8, 9) for y in range(-5, 6)]
        plt.plot(gx, gy, color='lightgray', marker='o', markersize=8, linestyle='none')

        for (x,y) in self.intersections:
            for i in range(8):
                inter = self.intersections[(x,y)]
                plt.plot([inter.x, inter.x + street_length[i][0]], [inter.y, inter.y + street_length[i][1]], color=street_color[inter.streets[i]])
                # Mark blocked streets with a black X at the midpoint
                if inter.blocked[i]:
                    mx = inter.x + street_length[i][0] / 2
                    my = inter.y + street_length[i][1] / 2
                    plt.plot(mx, my, color='black', marker='x', markersize=10, markeredgewidth=2)

        # Dijkstra tree overlay: small arrow at each intersection pointing along its next_heading
        for (x, y), inter in self.intersections.items():
            if inter.next_heading is not None:
                dx, dy = street_length[inter.next_heading]
                plt.arrow(inter.x, inter.y, dx*0.6, dy*0.6,
                          head_width=0.15, head_length=0.1,
                          color='orange', length_includes_head=True)

        # Safe zone rectangle drawn when a zone is active.
        if self.zone is not None:
            zxmin, zxmax, zymin, zymax = self.zone
            xs = [zxmin - 0.5, zxmax + 0.5, zxmax + 0.5, zxmin - 0.5, zxmin - 0.5]
            ys = [zymin - 0.5, zymin - 0.5, zymax + 0.5, zymax + 0.5, zymin - 0.5]
            plt.plot(xs, ys, color='pink', linestyle='--', linewidth=2)

        # Mark the chosen frontier (where directed-explore is headed) by recoloring its node.
        if self.chosen_frontier is not None:
            cx, cy = self.chosen_frontier
            plt.plot(cx, cy, color='red', marker='o', markersize=10)


    def show(self):
        self.draw_base()
        plt.pause(0.05)


    # inherits from draw base and adds the robot
    def show_with_robot(self, xbot, ybot, hbot):
        self.draw_base()

        arrow_length = 0.5

        rad = (45*hbot)*math.pi/180
        xbase = xbot + arrow_length/2 * math.sin(rad)
        ybase = ybot - arrow_length/2 * math.cos(rad)
        xtip = xbot - arrow_length/2 * math.sin(rad)
        ytip = ybot + arrow_length/2 * math.cos(rad)

        plt.arrow(xbase, ybase, xtip-xbase, ytip-ybase,
            width=0.2,
            head_width=0.3,
            head_length=0.1,
            color='magenta')

        plt.pause(0.05)


    # Build the optimal path/tree to (xgoal, ygoal) using Dijkstra
    # Stores cost + next_heading on every reachable Intersection
    def dijkstra(self, xgoal, ygoal, zone=None):
        # zone = (xmin, xmax, ymin, ymax): if given, never route outside it.
        # Reset tree data on every intersection before each run
        for inter in self.intersections.values():
            inter.cost = math.inf
            inter.next_heading = None

        # Set the goal: cost 0, no next-heading
        goal = self.getintersection(xgoal, ygoal)
        goal.cost = 0

        # create the on_deck array, initially containing only the goal
        on_deck = [goal]
        # set (no repeats) of all nodes that have been processed
        processed = set()

        while on_deck:

            # on_deck is kept sorted ascending by cost via bisect.insort below
            # so the cheapest leaf is always at index 0
            current = on_deck.pop(0)

            # add to processed set, once popped guaranteed optimal cost
            processed.add((current.x, current.y))

            # Sort through every outgoing street: only care about CONNECTED + not blocked
            for h in range(8):
                if current.streets[h] == Status.CONNECTED and not current.blocked[h]:
                    # facing that direction take a step forward to get to the neighbour
                    nx, ny = calc_move(current.x, current.y, h)
                    # Stay inside the zone if one was given (don't route out of it).
                    if zone is not None and not (zone[0] <= nx <= zone[1] and zone[2] <= ny <= zone[3]):
                        continue
                    neighbor = self.intersections[(nx, ny)]

                    # only check not procseed neighbors
                    if (neighbor.x, neighbor.y) not in processed:
                        # Even headings are cardinal odd are diagonal
                        if h % 2 == 0:
                            candidate_cost = current.cost + 1
                        else:
                            candidate_cost = current.cost + math.sqrt(2)

                        if candidate_cost < neighbor.cost:
                            # If neighbor was already on deck reinsert at the new sorted position
                            if neighbor.cost < math.inf:
                                on_deck.remove(neighbor)
                            neighbor.cost = candidate_cost
                            # neighbor points back to current by facing the opposite of h
                            neighbor.next_heading = calc_uturn(h)
                            bisect.insort(on_deck, neighbor)

        # No redraw here: the brain loop's show_with_robot() renders once per cycle.

    # Set blocked on (x,y,h) and mirror the flag onto the neighbor's opposite slot.
    def set_blocked(self, x, y, h, blocked):
        inter = self.getintersection(x, y)
        if inter.blocked[h] != blocked:
            inter.blocked[h] = blocked
            self.changed = True

        nx, ny = calc_move(x, y, h)
        opp = calc_uturn(h)
        neighbor = self.intersections.get((nx, ny))
        if neighbor is not None and neighbor.blocked[opp] != blocked:
            neighbor.blocked[opp] = blocked
            self.changed = True

        if blocked:
            now = time.time()
            self.block_times[(x, y, h)] = now
            if neighbor is not None:
                self.block_times[(nx, ny, opp)] = now
        else:
            self.block_times.pop((x, y, h), None)
            self.block_times.pop((nx, ny, opp), None)

    # Multitarget Dijkstra: seed each given intersection with its own initial cost and prop outward
    def dijkstra_multi(self, target_seeds):
        # Reset tree data on every intersection
        for inter in self.intersections.values():
            inter.cost = math.inf
            inter.next_heading = None

        # seed each target with its own starting cost
        on_deck = []
        for (tx, ty), seed_cost in target_seeds.items():
            if (tx, ty) not in self.intersections:
                continue
            inter = self.intersections[(tx, ty)]
            inter.cost = seed_cost
            bisect.insort(on_deck, inter)

        processed = set()

        while on_deck:
            current = on_deck.pop(0)
            processed.add((current.x, current.y))

            for h in range(8):
                if current.streets[h] == Status.CONNECTED and not current.blocked[h]:
                    nx, ny = calc_move(current.x, current.y, h)
                    neighbor = self.intersections[(nx, ny)]

                    if (neighbor.x, neighbor.y) not in processed:
                        if h % 2 == 0:
                            candidate_cost = current.cost + 1
                        else:
                            candidate_cost = current.cost + math.sqrt(2)

                        if candidate_cost < neighbor.cost:
                            if neighbor.cost < math.inf:
                                on_deck.remove(neighbor)
                            neighbor.cost = candidate_cost
                            neighbor.next_heading = calc_uturn(h)
                            bisect.insort(on_deck, neighbor)

        # No redraw here: the brain loop's show_with_robot() renders once per cycle.

    # Wipe Dijkstra tree data once goal reached
    def clear_dijkstra(self):
        for inter in self.intersections.values():
            inter.cost = math.inf
            inter.next_heading = None
        self.frontier_scores = {}
        self.chosen_frontier = None

    # Wipe every blocked flag. Used by the UI 'clear' command.
    def clear_blockages(self):
        cleared_any = False
        for inter in self.intersections.values():
            for h in range(8):
                if inter.blocked[h]:
                    cleared_any = True
                    inter.blocked[h] = False
        if cleared_any:
            self.changed = True

    def has_blockages(self):
        for inter in self.intersections.values():
            if any(inter.blocked):
                return True
        return False

    # Unblock any street not re-confirmed within max_age secs
    def expire_blockages(self, max_age):
        now = time.time()

        # which onees too old
        expired = []
        for (x, y, h), blocked_at in self.block_times.items():
            age = now - blocked_at
            if age > max_age:
                expired.append((x, y, h))

        # Then unblock expired
        for (x, y, h) in expired:
            self.set_blocked(x, y, h, False)

    # Pickle the whole Map object to filename
    def save(self, filename):
        print("Saving the map to %s..." % filename)
        with open(filename, 'wb') as file:
            pickle.dump(self, file)


# Load a pickled Map from filename and return it.
def load_map(filename):
    print("Loading the map from %s..." % filename)
    with open(filename, 'rb') as file:
        m = pickle.load(file)
    for inter in m.intersections.values():
        inter.blocked = [False, False, False, False, False, False, False, False]
        inter.nfc_id = None
    # Back-fill for maps pickled before the changed flag existed
    m.changed = False
    m.block_times = {}
    m.zone = None
    m.frontier_scores = {}
    m.chosen_frontier = None
    return m

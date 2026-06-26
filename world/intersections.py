from enum import Enum
import math

class Status(Enum): # name value pairs
    UNKNOWN = 0
    NONEXISTENT = 1
    UNEXPLORED = 2
    DEADEND = 3
    CONNECTED = 4


class Intersection():
   def  __init__(self, x, y):
      self.x = x
      self.y = y
      self.streets = [Status.UNKNOWN, Status.UNKNOWN, Status.UNKNOWN, Status.UNKNOWN, Status.UNKNOWN, Status.UNKNOWN, Status.UNKNOWN, Status.UNKNOWN]
      # One blocked flag per street. Independent of the street's status.
      self.blocked = [False, False, False, False, False, False, False, False]
      # Filled in by Map.dijkstra
      # -- cost = total cost from this node to the goal
      # -- next_heading = the heading at this node that follows the optimal tree toward the goal
      self.cost = math.inf
      self.next_heading = None
      # NFC tag id for this intersection, learned on first visit
      self.nfc_id = None

   # Sort by cost for queue
   def __lt__(self, other):
      return self.cost < other.cost



    
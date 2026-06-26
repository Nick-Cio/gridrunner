
# Set up a very simple ROS node to listen for goal/explore commands
#   and publish the current info (position, heading, intersection ID).
#
#   Node:       /PINAME         (this will use your Pi's name)
#
#   Publish:    ~/pose          geometry_msgs/PoseStamped
#   Subscribe:  ~/goal          geometry_msgs/Point
#               ~/explore       std_msgs/Empty
#
import ctypes
import json
import os
import rclpy
import socket
import time
import threading, prctl
import traceback

from math import pi, sin, cos, floor, ceil

from rclpy.node                 import Node
from rclpy.time                 import Time, Duration
from geometry_msgs.msg          import Point, PoseStamped, PolygonStamped
from std_msgs.msg               import Empty, String

from robot.ui import Mode


#
#   Simple ROS Node Class
#
class ROSNode(Node):
    # Initialization.
    def __init__(self, name, shared):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Save the shared data object.
        self.shared = shared

        # Create the publisher for the current information.
        self.pub = self.create_publisher(PoseStamped, '~/pose', 10)

        # Then create subscribers for goal and explore commands.
        self.create_subscription(Point, '~/goal',    self.cb_goal,    10)
        self.create_subscription(Empty, '~/explore', self.cb_explore, 10)
        self.create_subscription(Empty,          '~/stop', self.cb_stop, 10)
        self.create_subscription(PolygonStamped, '~/zone', self.cb_zone, 10)
        self.create_subscription(String,         '/loot',  self.cb_loot, 10)

        # Finally create a timer to drive the node.
        self.timer = self.create_timer(1.0, self.cb_timer)

        # Report and return.
        self.get_logger().info("ROS Node '%s' running" % (name))

    # Shutdown.
    def shutdown(self):
        # Destroy the timer and shut down the node.
        self.destroy_timer(self.timer)
        self.destroy_node()


    # Timer callback.  Send the robot's current pose.
    def cb_timer(self):
        # Grab the current x/y and heading from the shared data.
        if self.shared.acquire():
            posx  = self.shared.robotx
            posy  = self.shared.roboty
            head  = self.shared.robotheading
            iid   = self.shared.intersectionid
            self.shared.release()

        # Convert the heading into an angle (in radians).
        theta = pi/4 * float(head+2)

        # Make sure the intersection ID is a string.
        if iid is None:
            iid = 'Unknown inter ID'

        # Populate the message with the position/heading/iid and send.
        # The orientation is encoded as a quaternion.  The IID is a
        # string stored in the header frame_id.
        msg = PoseStamped()
        msg.header.frame_id    = iid
        msg.pose.position.x    = float(posx)
        msg.pose.position.y    = float(posy)
        msg.pose.orientation.z = sin(theta/2)
        msg.pose.orientation.w = cos(theta/2)
        self.pub.publish(msg)


    # Goal command callback.
    def cb_goal(self, msg):
        # Extract the goal coordinates from the message.
        xgoal = int(msg.x)
        ygoal = int(msg.y)

        # Report.
        self.get_logger().info("Received goal command (%d,%d)" % (xgoal,ygoal))

        # Same thing the UI 'G' command does. 
        # Set the mode and goal coords together under the lock.
        if self.shared.acquire():
            self.shared.mode = Mode.GOTO
            self.shared.xgoal = xgoal
            self.shared.ygoal = ygoal
            self.shared.release()

    # Explore command callback.
    def cb_explore(self, msg):
        # Report.
        self.get_logger().info("Received explore command")

        # Same thing the UI 'E' command does.
        if self.shared.acquire():
            self.shared.mode = Mode.EXPLORE
            self.shared.release()

    # Stop command callback.
    def cb_stop(self, msg):
        # Report.
        self.get_logger().info("Received stop command")

        # Same thing the UI 'S' command does.
        if self.shared.acquire():
            self.shared.stop = True
            self.shared.release()

    # Zone command callback.
    def cb_zone(self, msg):
        # Make sure the message contains 4 points (rectangle).
        if len(msg.polygon.points) != 4:
            self.get_logger().warn("Received BAD zone command")
            return

        # Extract the information from the message.
        x_top_left  = msg.polygon.points[0].x
        y_top_left  = msg.polygon.points[0].y
        x_top_right = msg.polygon.points[1].x
        y_top_right = msg.polygon.points[1].y
        x_bot_right = msg.polygon.points[2].x
        y_bot_right = msg.polygon.points[2].y
        x_bot_left  = msg.polygon.points[3].x
        y_bot_left  = msg.polygon.points[3].y

        # Understand the timing.
        t_active = Time().from_msg(msg.header.stamp)
        t_now    = self.get_clock().now()

        seconds_togo = (t_active - t_now).nanoseconds / 1e9

        # Report.
        self.get_logger().info(
            ("Received zone command: in %4.1fs - " +
             "(%.1f,%.1f), (%.1f,%.1f), (%.1f,%.1f), (%.1f,%.1f)") %
            (seconds_togo,
             x_top_left,  y_top_left,  x_top_right, y_top_right,
             x_bot_right, y_bot_right, x_bot_left,  y_bot_left))

        # Corners sit at +/- 0.5, so round inward to whole intersections.
        xmin = int(ceil(min(x_top_left, x_top_right, x_bot_right, x_bot_left)))
        xmax = int(floor(max(x_top_left, x_top_right, x_bot_right, x_bot_left)))
        ymin = int(ceil(min(y_top_left, y_top_right, y_bot_right, y_bot_left)))
        ymax = int(floor(max(y_top_left, y_top_right, y_bot_right, y_bot_left)))

        # Place in shared data! Same thing the UI 'Z' command does.
        if self.shared.acquire():
            self.shared.mode = Mode.ZONE
            self.shared.zone_xmin = xmin
            self.shared.zone_xmax = xmax
            self.shared.zone_ymin = ymin
            self.shared.zone_ymax = ymax
            self.shared.zone_treach = seconds_togo
            self.shared.release()

    # Loot dictionary callback.
    def cb_loot(self, msg):
        # Grab the raw message data.
        raw_data = json.loads(msg.data)

        # Build the dictionary: {'0x123456': {distance: 5, name: 'xyz'}, ...}
        loot = {}
        for iid in raw_data:
            loot[iid] = {}
            for info in raw_data[iid]:
                loot[iid][info] = raw_data[iid][info]

        # Report.
        #self.get_logger().info("Received loot dictionary")
        #if loot:
        #    self.get_logger().info("LOOT: %s" % loot)

        # Place in shared data.
        if self.shared.acquire():
            self.shared.loot = loot
            self.shared.release()



#
#   Main ROS Thread Code
#
def runros(shared):
    # Expose the thread name to Linux.
    prctl.set_name(threading.current_thread().name)

    # Setup network access for ROS on domain #1.
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='SUBNET'
    os.environ['ROS_DOMAIN_ID']='1'

    # Initialize ROS.
    rclpy.init()

    # Instantiate a simple ROS node, named after the host name, and
    # passing the shared data object.
    node = ROSNode(socket.gethostname(), shared)

    # Spin the node until interrupted.
    try:
        rclpy.spin(node)
    except BaseException as ex:
        print("Ending Run-ROS due to exception: %s" % repr(ex))
        # traceback.print_exc()

    # Shutdown the node and ROS.  The external shutdown already causes this.
    #node.shutdown()
    #rclpy.shutdown()



######################################################################
#
#   Testing Code
#
#   This executes if run stand-alone.  Similar functionality should be
#   in your existing code.
#
if __name__ == "__main__":

    ### Dummy shared data object and run UI function for testing:

    # Define the shared data here for testing.
    class SharedData:
        # Initialize the data.
        def __init__(self):
            # Create the lock.
            self.lock = threading.Lock()

            # Initialize the data.  The following are example data:
            self.robotx         = 24
            self.roboty         = 35
            self.robotheading   = 3
            self.intersectionid = '0x1234567890abcd'

        # Method to acquire/gain access to the shared data.
        def acquire(self):
            return self.lock.acquire()

        # Method to release/relinquish access to the shared data.
        def release(self):
            self.lock.release()

    # Define a dummy robot function for testing.
    def runrobot(shared):
        # Just sleep a lot.  You would have the robot code here.
        try:
            while True:
                time.sleep(1)

        # If signaled, break out of the infinite loop and return.
        except BaseException as ex:
            print("Ending Run-Robot due to exception: %s" % repr(ex))
            # traceback.print_exc()


    ### Dummy code for testing:

    # Instantiate the shared data.
    shared = SharedData()

    # Run the ROS thread - notice the argument.
    rosthread = threading.Thread(
        name="ROSThread", target=runros, args=(shared,))
    rosthread.start()

    # Run the ROBOT thread.
    runrobot(shared)

    # Send an exception (Keyboard Interrupt) to the ROS thread
    # to finish. Then wait to join.
    print("Interrupting ROS thread...")
    ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(rosthread.ident),
        ctypes.py_object(KeyboardInterrupt))
    rosthread.join()

    # Nothing to shutdown
    print("Exiting")

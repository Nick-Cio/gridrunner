# GridRunner

Autonomous mapping and navigation code for a small differential-drive robot. This
was the final project for ME129 (Robotics) at Caltech.

The robot drives around a grid of black-tape streets that it has never seen before,
figures out where the intersections are, and builds a map as it goes. Once it knows
part of the map it can plan a route to any point it has already found, head toward a
target zone, or keep exploring on its own.

## What it does

* Follows lines and detects intersections using an infrared sensor array.
* Tracks heading by integrating a gyroscope, and reads NFC tags placed at
  intersections to check where it is.
* Stores everything it learns as a graph of intersections and the streets between
  them. Each street is marked as connected, unexplored, a dead end, or nonexistent.
* Plans routes with Dijkstra's algorithm over that graph.
* Picks where to explore next by scoring unexplored frontiers against how far they
  are to drive to and how close they sit to the current goal.
* Avoids obstacles with ultrasonic sensors and backs out of dead ends or blocked
  streets.
* Takes commands and reports its position over ROS, with a keyboard interface for
  manual driving and setting goals.

## Layout

```
main.py                 entry point, starts the ROS thread and the main loop
ros.py                  ROS interface (runs on its own thread)
robot/
  brain.py              main control loop, exploration and planning logic
  config.py             tunable constants (speeds, thresholds, costs)
  ui.py                 keyboard commands and shared state between threads
  utils.py              grid and heading helpers
behaviors/
  BasicBehaviors.py     line following, turning, pulling through intersections
  herding.py            herding behavior
  wall_following.py     wall following behavior
controllers/
  drive_system.py       motor control
  gyro.py               gyro reading and zeroing
  line_sensors.py       intersection and line event detectors
  nfc.py                NFC tag reader
  proximity_sensor.py   ultrasonic distance and blocked checks
hardware/
  adc.py                analog to digital converter
  ir_sensor.py          infrared line sensor
  motor.py              motor driver
  ultrasound.py         ultrasonic sensor
world/
  map.py                graph map and Dijkstra
  intersections.py      intersection and street model
map1 .. map4            saved maps
files/                  more saved maps
tests/
  flower_power.py       live tuning tool for turn and pull-through timing
```

## Running

The code runs on the robot's Raspberry Pi, which has the motors, sensors, and ROS
set up. From the project folder:

```
python3 main.py
```

It starts in manual mode. Drive with the keyboard, set a goal or a zone, or let it
explore. Press Q to quit, which saves the map and shuts the motors down.

## Notes

The hardware imports (sensors, motors, ROS) only work on the Pi, so the code will not
run as is on a normal computer.

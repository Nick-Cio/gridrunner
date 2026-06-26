# Centralized pin assignments and tuning constants for SIGMA JARVIS


# --- Motor GPIO Pins ---
# Each motor uses a forward pin and a backward pin for H-bridge control.
MOTOR_R_FORWARD_PIN = 8
MOTOR_R_BACKWARD_PIN = 7
MOTOR_L_FORWARD_PIN = 6
MOTOR_L_BACKWARD_PIN = 5

# --- IR Sensor GPIO Pins ---
PIN_IR_LEFT = 14
PIN_IR_MIDDLE = 15
PIN_IR_RIGHT = 18

# --- ADC GPIO Pins ---
ADC_CLOCK_PIN = 17
ADC_BIT_PINS = (11, 9, 25, 10, 24, 22, 23, 27)  # bit7 .. bit0
ADC_READ_DELAY_S = 150e-6

# --- Gyroscope ---
GYRO_SCALE = 0.0881  # rad/s per ADC count
GYRO_RESETZERO_SAMPLES = 20
GYRO_RESETZERO_INTERVAL_S = 0.01
GYRO_RESET_SETTLE_S = 0.25  # brake/post-reset settle around resetzero in turn()

# --- Ultrasound ---
ULTRA_LEFT_TRIG = 13
ULTRA_LEFT_ECHO = 16
ULTRA_MIDDLE_TRIG = 19
ULTRA_MIDDLE_ECHO = 20
ULTRA_RIGHT_TRIG = 26
ULTRA_RIGHT_ECHO = 21

SPEED_OF_SOUND = 350           # m/s
ULTRASOUND_INIT_DISTANCE = 10.0  # meters

# --- PWM Settings ---
PWM_RANGE = 255
PWM_FREQUENCY = 1000


# --- Drive Tuning ---
# Maps a turning degree index to (left_level, right_level).
# Separate tables for right and left turns allow independent tuning.

# Used when direction = 1 (turning right)
DEGREE_LEVEL_RIGHT = {
    0: (0.6, 0.63),     # Straight
    1: (0.70, 0.75),    # Gentle (~45 deg)
    2: (0.74, 0.66),    # Medium (~90 deg)
    3: (0.7, 0.48),     # Sharp (~180 deg)
    4: (0.7, 0),        # Hook turn
    5: (0.57, -0.57),   # Spin in place
}

# Used when direction = -1 (turning left)
DEGREE_LEVEL_LEFT = {
    0: (0.6, 0.63),     # Straight
    1: (0.65, 0.72),    # Gentle (~45 deg)
    2: (0.64, 0.77),    # Mediulm (~90 deg)
    3: (0.42, 0.78),    # Sharp (~180 deg)
    4: (0, 0.73),       # Hook turn
    5: (-0.57, 0.57),   # Spin in place
}

# Pull-forward (slow motion used after intersection detection).
PULL_FORWARD_LEFT_LEVEL = 0.70
PULL_FORWARD_RIGHT_LEVEL = 0.75

# Sensor reading -> (degree, direction) lookup for line follower
SENSOR_ACTION = {
    (0, 1, 0): (0,  1),   # Center on line: straight
    (0, 1, 1): (3,  1),   # Drifting right: medium right
    (0, 0, 1): (4,  1),   # Far right: sharp right
    (1, 1, 0): (3, -1),   # Drifting left: medium left
    (1, 0, 0): (4, -1),   # Far left: sharp left
    (1, 1, 1): (0,  1),   # All black: straight
}


# --- Detector Tuning ---
# EMA time constants. Smaller T = faster response but also more sensitive.
TURN_DETECTOR_T = 0.1
SIDE_DETECTOR_T = 0.08
INTERSECTION_DETECTOR_T = 0.1
END_DETECTOR_T = 0.1
TURN_OFF_LINE_T = 0.025

# Detector fires when its EMA state crosses the threshold.
TURN_DETECTOR_THRESHOLD = 0.50
SIDE_DETECTOR_THRESHOLD = 0.50
INTERSECTION_DETECTOR_THRESHOLD = 0.60
END_DETECTOR_THRESHOLD = 0.58
TURN_OFF_LINE_THRESHOLD = 0.3


# --- Behavior Tuning ---

# Turn
TURN_TIMEOUT_S = 5
TURN_OVERSHOOT_S = 0.2   # after finding the new line, keep turning this long before recentering

# Line follower
INTERSECTION_PAUSE_S = 0.6
INTERSECTION_PULL_FORWARD_S = 0.30

# Brain timings
DEADEND_BRAKE_PAUSE_S = 0.15      # settle after braking post U-turn
MAP_REFRESH_INTERVAL_S = 0.5      # idle redraw rate after exploration completes
BLOCKAGE_TTL_S = 120               # forget a blockage if not re-seen within this many seconds
TURN_SETTLE_S = 0.25              # rest after a turn so ultrasound buckets fill before reading

# Stuck guard
STUCK_TURN_LIMIT = 3              # turns with no forward progress before parking and waiting
MIDSTREET_UTURN_LIMIT = 4        # mid-street blockage u-turns before waiting it out
MIDSTREET_UTURN_PAUSE_S = 0.6    # pause between mid-street u-turn attempts

# Directed explore seed weights
UNEXPLORED_TRAVERSE_COST = 1.5   # extra cost to drive an unexplored street (might dead-end)
UNKNOWN_TRAVERSE_COST = 3.5      # extra cost to scan an unknown slot (might not be a road at all)
CARTESIAN_PENALTY = 2.0          # weight on straight-line distance from the landed cell to goal

# Spiral search
SPIRAL_FAST_LEVEL = 0.85
SPIRAL_SLOW_LEVEL_INITIAL = 0.3
SPIRAL_STEP_INTERVAL_S = 1.0
SPIRAL_SLOW_INCREMENT = 0.05
SPIRAL_MIN_LEVEL_GAP = 0.1

# Herding
HERD_TRIGGER_INTERVAL_S = 0.050
HERD_SIDE_THRESHOLD_M = 0.20    # side sensor: obstacle if closer than this
HERD_FRONT_CLEAR_M = 0.20       # middle sensor: > this -> advance
HERD_FRONT_BACKUP_M = 0.10      # middle sensor: < this -> back up
HERD_TURN_DEGREE = 3            # sharp curve
HERD_SPIN_DEGREE = 5            # spin in place

# Wall following
WALL_TRIGGER_INTERVAL_S = 0.050
WALL_NOMINAL_DISTANCE_M = 0.30   # target distance from the wall
WALL_STOP_ERROR_M = 0.20         # if error above this -> brake
WALL_FRONT_STOP_M = 0.20         # obstacle ahead this close -> brake

# Proportional wall-following
# PWM_l = WALL_C_L + WALL_K_L * error
# PWM_r = WALL_C_R + WALL_K_R * error
WALL_C_L = 0.64
WALL_K_L = -1.5
WALL_C_R = 0.69
WALL_K_R = 1.0
# Error-band thresholds (in choose_action()):
#   < 2 cm  -> straight (deg 0)
#   < 5 cm  -> gentle   (deg 1)
#   < 8 cm  -> medium   (deg 2)
#   < 10 cm -> sharp    (deg 3)

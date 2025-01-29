"""csci3302_lab2 controller."""

import math
from controller import Robot, Motor, DistanceSensor

# ---------------------------
# User-Tuned Global Constants
# ---------------------------
# Ground sensor threshold to distinguish black line vs white floor
GROUND_SENSOR_THRESHOLD = 600

# ePuck specs (approx)
EPUCK_AXLE_DIAMETER = 0.053    # 53 mm between wheels
EPUCK_MAX_WHEEL_SPEED = 0.13   # ~0.13 m/s at MAX_SPEED = 6.28 rad/s
MAX_SPEED = 6.28               # [rad/s] as set by Webots for ePuck

# Initialize pose (in Webots' coordinate system)
pose_x = 0.0
pose_y = 0.0
pose_theta = 0.0

# Ground Sensor Indices
LEFT_IDX = 0
CENTER_IDX = 1
RIGHT_IDX = 2

# Create the Robot instance
robot = Robot()
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))   # Velocity control mode
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and enable ground sensors
ground_sensors = [robot.getDevice('gs0'), 
                  robot.getDevice('gs1'), 
                  robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Let sensors stabilize
for _ in range(10):
    robot.step(SIM_TIMESTEP)

# ---------------------------
# Variables for line following
# ---------------------------
vL = 0.0  # Wheel speed left (Webots units, i.e. 0..MAX_SPEED)
vR = 0.0  # Wheel speed right

# Loop-closure detection
line_encountered_once = False
start_pose_x = 0.0
start_pose_y = 0.0
start_pose_theta = 0.0

# ---------------------------
# Main Control Loop
# ---------------------------
while robot.step(SIM_TIMESTEP) != -1:

    # Read ground sensor values
    gsr = [gs.getValue() for gs in ground_sensors]
    # print(gsr)  # Uncomment for debugging

    # ------------------------------------------------------
    # 1) Simple Line-Following Logic
    # ------------------------------------------------------
    left_black   = (gsr[LEFT_IDX]   < GROUND_SENSOR_THRESHOLD)
    center_black = (gsr[CENTER_IDX] < GROUND_SENSOR_THRESHOLD)
    right_black  = (gsr[RIGHT_IDX]  < GROUND_SENSOR_THRESHOLD)

    # Default to going straight
    vL = 0.5 * MAX_SPEED
    vR = 0.5 * MAX_SPEED

    # If the center sensor is black, we are on the line → go straight
    # If left sensor is black, turn left
    # If right sensor is black, turn right
    if left_black and not center_black:
        # Turn left
        vL = 0.2 * MAX_SPEED
        vR = 0.5 * MAX_SPEED
    elif right_black and not center_black:
        # Turn right
        vL = 0.5 * MAX_SPEED
        vR = 0.2 * MAX_SPEED
    elif center_black:
        # Go straight (could optionally tweak speed)
        vL = 0.5 * MAX_SPEED
        vR = 0.5 * MAX_SPEED
    else:
        # If no sensor sees black, we might be “lost.”
        # Strategy: reduce speed or spin to search for line
        vL = 0.3 * MAX_SPEED
        vR = 0.0 * MAX_SPEED
        # or do something else based on your scenario

    # ------------------------------------------------------
    # 2) Odometry Update
    # ------------------------------------------------------
    # Convert wheel speeds from Webots rad/s to linear m/s
    v_left_m_s  = (vL / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED
    v_right_m_s = (vR / MAX_SPEED) * EPUCK_MAX_WHEEL_SPEED

    dt = SIM_TIMESTEP / 1000.0  # Convert ms to s

    # Calculate forward speed and angular velocity
    v = (v_left_m_s + v_right_m_s) / 2.0
    omega = (v_right_m_s - v_left_m_s) / EPUCK_AXLE_DIAMETER

    # Update pose
    pose_theta += omega * dt
    # Depending on your coordinate convention, in Webots the X-axis 
    # often points “down” the arena and Y-axis to the robot’s right. 
    # For standard robotics, you might treat x as forward, y as left, etc.
    # Adjust if needed. For a conventional “x-forward, y-left”:
    pose_x += v * math.cos(pose_theta) * dt
    pose_y += v * math.sin(pose_theta) * dt

    # ------------------------------------------------------
    # 3) Loop Closure Code
    # ------------------------------------------------------
    # Example: if center sensor sees black, we’re crossing the line
    if center_black:
        if not line_encountered_once:
            # First time we see the line
            line_encountered_once = True
            start_pose_x = pose_x
            start_pose_y = pose_y
            start_pose_theta = pose_theta
        else:
            # We have seen the line before -- we just closed the loop
            loop_error_x = pose_x - start_pose_x
            loop_error_y = pose_y - start_pose_y
            loop_error_theta = pose_theta - start_pose_theta

            print("Loop closed!")
            print(f"  Loop error in X = {loop_error_x:.4f} m")
            print(f"  Loop error in Y = {loop_error_y:.4f} m")
            print(f"  Loop error in Theta = {loop_error_theta:.4f} rad")

            # Optionally reset for repeated loops:
            line_encountered_once = False
    else:
        # We’re not currently on the line
        pass

    # Print out current pose for debugging
    print("Current pose: [{:.4f}, {:.4f}, {:.4f}]".format(pose_x, pose_y, pose_theta))

    # Send commands to motors
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)

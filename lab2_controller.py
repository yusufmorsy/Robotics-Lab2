"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor
# import os

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
GROUND_SENSOR_THRESHOLD = 390 #below 500 means black

# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0

# Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
LEFT_IDX = 0
CENTER_IDX = 1
RIGHT_IDX = 2

# create the Robot instance.
robot = Robot()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.13 # m/s
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)  

vL = 0
vR = 0

reset_timer = 0
state = "drive_forward"

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:

    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    #sees a line under the center sensor
    if gsr[1] < GROUND_SENSOR_THRESHOLD: 
        state = "drive_forward"

    #sees a line under the left sensor
    if gsr[0] < GROUND_SENSOR_THRESHOLD: 
        state = "rotate_left"

    #sees a line under the right sensor
    if gsr[2] < GROUND_SENSOR_THRESHOLD: 
        state = "rotate_right"

    #if it can't find a line
    if gsr[0] > GROUND_SENSOR_THRESHOLD and gsr[1] > GROUND_SENSOR_THRESHOLD and gsr[2] > GROUND_SENSOR_THRESHOLD: 
        state = "rotate_left"

    #if it's hit the start line
    if gsr[1] < GROUND_SENSOR_THRESHOLD and gsr[0] < GROUND_SENSOR_THRESHOLD and gsr[1] < GROUND_SENSOR_THRESHOLD and gsr[2] < GROUND_SENSOR_THRESHOLD: 
        reset_timer = reset_timer - 1
        
        #only reset odometry if seeing the start line for 3 consecutive frames
        state = "drive_forward"
        if reset_timer < 0:
            pose_x = 0
            pose_y = 0
            pose_theta = 0
    else:
        reset_timer = 3

    #update wheel speeds and odometry
    if state == "drive_forward":
        vL = MAX_SPEED
        vR = MAX_SPEED
        pose_x  = pose_x + math.cos(pose_theta) * EPUCK_MAX_WHEEL_SPEED
        pose_y  = pose_y - math.sin(pose_theta) * EPUCK_MAX_WHEEL_SPEED
    elif state == "rotate_left":
        vL = 0
        vR = MAX_SPEED
        pose_theta = pose_theta + 0.0246
    elif state == "rotate_right":
        vL = MAX_SPEED
        vR = 0
        pose_theta = pose_theta - 0.0246
    
    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)

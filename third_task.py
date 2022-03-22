#!/usr/bin/env python3
import string
from ev3dev.ev3 import *
import time
import math

RADIUS_OF_WHEEL = 0.025
DISTANCE_BETWEEN_WHEELS = 0.15
PI = math.pi
DEG2RAD = PI / 180
ERROR = 0.5

a = 0
b = 1
c = 1
planed_time = 0


leftMotor = LargeMotor('outA')
rightMotor = LargeMotor('outD')

prevLeftAngel, prevRightAngel = 0, 0
leftMotor.position = rightMotor.position = 0  # reset memory

# Coefficients for P controller 
K_STRAIGHT = 300
K_ROTATION = 300

# List of target or single target

# (0.5; 0) -> (0, 0.8) -> (-1.2; 0) -> (0, -1)
targets =   [[0.5, 0], [0,0], 
            [0,0.8], [0,0], 
            [-1.2, -0], [0,0], 
            [0, -1], [0,0]]

# Define some innitial value of robot
xCoord, yCoord = 0, 0 # This is initial coordinate of robot 
course = 0 # This is initial course angle of robot (theta). The angle between vector linear velocity and Ox axis.
startTime = time.time() # This is initial time 
currentTime = startTime # Set the first value for currentTime


# Function to write data with format "x y time":
def writeData(xCoord, yCoord, currentTime):
    sh.write(str(xCoord) + " " + str(yCoord) + " " + str(currentTime) + "\n")


#Function to calculate target
def idea_trajectory(a, b, c, t):
    x = a + b*math.sin(2*c*t)
    y = b*math.sin(c*t)
    return x, y

sh = open("Data/third_task_data_ks="+str(K_STRAIGHT)+"_kr="+str(K_ROTATION)+"_a="+str(a)+"_b="+str(b)+"_c="+str(c)+".txt", "w") # Open the file
writeData(xCoord, yCoord, currentTime) # Write to file for the first time.

while (1): # This is a loop, which go through every target in array targets.
    
    # Assign xGoal and yGoal
    xGoal, yGoal = idea_trajectory(a, b, c, planed_time)
    planed_time+=0.2

    while (1):

        # Update currentTime base on system time tick
        currentTime = time.time() - startTime

        # Get angle of 2 motors (ψ) and convert them to RAD (for easier calculation)
        curLeftAngle = leftMotor.position * DEG2RAD   
        curRightAngle = rightMotor.position * DEG2RAD
        
        # Calculate the change of motors's angle after a cycle 
        dRightAngle = curRightAngle - prevRightAngel
        dLeftAngle = curLeftAngle - prevLeftAngel

        # Assign previous motor's angle to current motor's angle
        prevLeftAngel, prevRightAngel = curLeftAngle, curRightAngle
        
        # Update the course angle (theta). (V; Ox). But acorrding to the formula, we need a previous course angle which need to be added.
        course = (curRightAngle - curLeftAngle) * RADIUS_OF_WHEEL / DISTANCE_BETWEEN_WHEELS

        # Update coordinates. But acorrding to the formula, we need previous coordinates here too!!! What the fuck is happening???
        xCoord += math.cos(course) * (dRightAngle + dLeftAngle) * RADIUS_OF_WHEEL / 2
        yCoord += math.sin(course) * (dRightAngle + dLeftAngle) * RADIUS_OF_WHEEL / 2

        # Calculate the distance between current's x, y and goal's x, y
        deltaX, deltaY = xGoal - xCoord, yGoal - yCoord

        # Calculating distance to goal coordinate (tính khoảng cách từ vị trí hiện tại đến vị trí đích)
        distance = math.sqrt(deltaX * deltaX + deltaY * deltaY)

        # Update bearing (góc giữa Ox và vector chỉ từ vị trí hiện tại đến vị trí đích)
        bearing = math.atan2(deltaY, deltaX)

        # Update heading (angle alpha) 
        heading = bearing - course

        # shortest corner turn
        if abs(heading) > PI:
            heading -= math.copysign(1, heading) * 2 * PI


        # calculation of linear speed
        baseSpeed = K_STRAIGHT * distance * math.cos(heading)
        if abs(baseSpeed) > 50:
            baseSpeed = math.copysign(1, baseSpeed) * 50

        # calculation of angular velocity
        control = K_STRAIGHT*math.cos(heading)*math.sin(heading) +  K_ROTATION * heading 
        if abs(control) > 30:
            control = math.copysign(1, control) * 30

        # control conversion from float to integer, PWM percentage
        pwmRight = baseSpeed + control
        pwmLeft = baseSpeed - control

        if (abs(pwmLeft) > 100):
            pwmLeft = math.copysign(1, pwmLeft) * 100
        if (abs(pwmRight) > 100):
            pwmRight = math.copysign(1, pwmRight) * 100

        leftMotor.run_direct(duty_cycle_sp=int(pwmLeft))
        rightMotor.run_direct(duty_cycle_sp=int(pwmRight))

        # Write data
        writeData(xCoord, yCoord, currentTime)

        # exit when reaching goal area
        if (distance < ERROR):
            #leftMotor.stop(stop_action='brake')
            #rightMotor.stop(stop_action='brake')
            #sh.close()
            sh.write("\n\n\n")
            break
            
sh.close()

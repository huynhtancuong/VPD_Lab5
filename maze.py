#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

# Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
motorD = LargeMotor(OUTPUT_D)
motorC = LargeMotor(OUTPUT_C)
front_left_motor = motorA
front_right_motor = motorB
rear_left_motor = motorD
rear_right_motor = motorC 


tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()
btn = Button()
radio = Radio()

color_sensor_in1 = ColorSensor(INPUT_1)
ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2)
ultrasonic_sensor_in3 = UltrasonicSensor(INPUT_3)
ultrasonic_sensor_in4 = UltrasonicSensor(INPUT_4)
ultrasonic_sensor_in5 = UltrasonicSensor(INPUT_5)
gyro_sensor_in6 = GyroSensor(INPUT_6)
pen_in7 = Pen(INPUT_7)

motorC = LargeMotor(OUTPUT_C) # Wheel Actuator
motorD = LargeMotor(OUTPUT_D) # Wheel Actuator

# Here is where your code starts
def getFrontLeftDistance():
	ultrasonic_sensor_in3.distance_centimeters
	frontLeftDistanceCm = ultrasonic_sensor_in3.distance_centimeters
	return frontLeftDistanceCm

def getFrontRightDistance():
	ultrasonic_sensor_in2.distance_centimeters
	frontRightDistanceCm = ultrasonic_sensor_in2.distance_centimeters
	return frontRightDistanceCm

def getFrontDistance():
	average = (getFrontRightDistance() + getFrontLeftDistance()) / 2
	return average

def getLeftDistance():
	ultrasonic_sensor_in5.distance_centimeters
	leftDistanceCm = ultrasonic_sensor_in5.distance_centimeters
	return leftDistanceCm

def getRightDistance():
	ultrasonic_sensor_in4.distance_centimeters
	rightDistanceCm = ultrasonic_sensor_in4.distance_centimeters
	return rightDistanceCm

def turnLeft():
    K_p = 3
    counter = 0
    error = 2
    gyro_sensor_in6.reset()

    while True:
        
        angle = gyro_sensor_in6.angle
        # print(angle)
        
        error_state = -90 - angle
        base = K_p * error_state
        if abs(base) > 100:
            base = math.copysign(1, base)*100
        print(base)
        print("counter" +str(counter) )
        
        front_left_motor.on(base/2)
        rear_left_motor.on(base/2)
        front_right_motor.on(-base/2)
        rear_right_motor.on(-base/2)
        print("True")
        
        if counter >= 3:
            print("stop turn left")
            break
        if abs(error_state) < error:
            counter +=1
            continue
        if abs(error_state) >= error:
            counter = 0
    front_left_motor.off(brake=True)
    rear_left_motor.off(brake=True)
    front_right_motor.off(brake=True)
    rear_right_motor.off(brake=True)

def turnRight():
    K_p = 3
    counter = 0
    error = 2
    gyro_sensor_in6.reset()
    while True:
        
        angle = gyro_sensor_in6.angle
        # print(angle)
        
        error_state = 90 - angle
        base = K_p * error_state
        if abs(base) > 100:
            base = math.copysign(1, base)*100
        print(base)
        print("counter" +str(counter) )
        
        front_left_motor.on(base/2)
        rear_left_motor.on(base/2)
        front_right_motor.on(-base/2)
        rear_right_motor.on(-base/2)
        print("True")
        
        if counter >= 3:
            print("stop turn left")
            break
        if abs(error_state) < error:
            counter +=1
            continue
        if abs(error_state) >= error:
            counter = 0
    
    front_left_motor.off(brake=True)
    rear_left_motor.off(brake=True)
    front_right_motor.off(brake=True)
    rear_right_motor.off(brake=True)

def limitSpeedUpFunction(t, T):
    return (1 - math.exp(-t/T))

def forward():
    K_p = 3
    counter = 0
    error = 0.3
    
    start_time = time.time()
	
    while True:
	    
    	distance = getFrontDistance()
    	
    	error_state = 10 - distance
    	
    	base = K_p * error_state
        
        frontBase, rearBase = base, base
        
        counter_time = time.time() - start_time
        frontLimit = 100*limitSpeedUpFunction(counter_time, 0.17)
    	if abs(frontBase) > frontLimit:
            frontBase = math.copysign(1, base)*frontLimit
            
        raerLimit = 100*limitSpeedUpFunction(counter_time, 0.25)
    	if abs(frontBase) > raerLimit:
            rearBase = math.copysign(1, base)*raerLimit
        
        front_left_motor.on(-frontBase)
        rear_left_motor.on(-rearBase)
        front_right_motor.on(-frontBase)
        rear_right_motor.on(-rearBase)
        print(base)
        print("counter" +str(counter) )
        if counter >= 3:
            print("stop turn left")
            break
        if abs(error_state) < error:
            counter +=1
            continue
        if abs(error_state) >= error:
            counter = 0
            
    front_left_motor.off(brake=True)
    rear_left_motor.off(brake=True)
    front_right_motor.off(brake=True)
    rear_right_motor.off(brake=True)	
    	
pen_in7.down()
turnRight()
forward()
turnLeft()
forward()
    	
    	
    	
    	
    	
    	
    	
    	
    	
    	
    	
    	
    	
	

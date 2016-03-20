#!/usr/bin/env python3

import subprocess
import time
import math
import ev3dev.auto as ev3
from collections import deque

########################################################################
##
## Sensor Setup
##
########################################################################

# Using string names, will include this in ev3dev.brickpi soon!
p = ev3.LegoPort(address='ttyAMA0:S1')
p.mode = 'nxt-analog'
p.set_device = 'ht-nxt-gyro'

p = ev3.LegoPort(address='ttyAMA0:S2')
p.mode = 'nxt-analog'
p.set_device = 'lego-nxt-touch'

touchSensor = ev3.TouchSensor('ttyAMA0:S2')
gyro_sensor = ev3.Sensor('ttyAMA0:S1')
#gyro_sensor.mode = gyro_sensor.MODE_GYRO_RATE


########################################################################
##
## Motor Setup
##
########################################################################


left_motor = ev3.Motor('ttyAMA0:MC')
right_motor = ev3.Motor('ttyAMA0:MB')

motors = [left_motor,right_motor]

for motor in motors:
    # Reset all of the motor parameter attributes to their default values.
    # This will also have the effect of stopping the motor.
    motor.reset()

    # Run the motor using the duty cycle specified by duty_cycle_sp. Unlike other run commands,
    # changing duty_cycle_sp while running will take effect immediately.
    #motor.run_direct()

time.sleep(0.01)

########################################################################
##
## Definitions and Initialization variables
##
########################################################################    
                
           
#Timing settings for the program
loopTimeMiliSec         = 10                    # Time of each loop, measured in miliseconds.
loopTimeSec             = loopTimeMiliSec/1000.0# Time of each loop, measured in seconds.
motorAngleHistoryLength = 4                     # Number of previous motor angles we keep track of.
loopCount               = 1                     # Loop counter, starting at 1

#Math constants
radiansPerDegree               = math.pi/180                                   # The number of radians in a degree.

#Platform specific constants and conversions
degPerSecondPerRawGyroUnit     = 1                                             # For the LEGO EV3 Gyro in Rate mode, 1 unit = 1 deg/s
radiansPerSecondPerRawGyroUnit = degPerSecondPerRawGyroUnit*radiansPerDegree   # Express the above as the rate in rad/s per gyro unit
degPerRawMotorUnit             = 1                                             # For the LEGO EV3 Large Motor 1 unit = 1 deg
radiansPerRawMotorUnit         = degPerRawMotorUnit*radiansPerDegree           # Express the above as the angle in rad per motor unit
RPMperPerPercentSpeed          = 1.7                                           # On the EV3, "1% speed" corresponds to 1.7 RPM (if speed control were enabled)
degPerSecPerPercentSpeed       = RPMperPerPercentSpeed*360/60                  # Convert this number to the speed in deg/s per "percent speed"
radPerSecPerPercentSpeed       = degPerSecPerPercentSpeed * radiansPerDegree   # Convert this number to the speed in rad/s per "percent speed"

# The rate at which we'll update the gyro offset (precise definition given in docs)
gyroDriftCompensationRate      = 0.1*loopTimeSec*radiansPerSecondPerRawGyroUnit

# A deque (a fifo array) which we'll use to keep track of previous motor positions, which we can use to calculate the rate of change (speed)
motorAngleHistory = deque([0],motorAngleHistoryLength)

# State feedback control gains (aka the magic numbers)
gainGyroAngle                  = 1156  # For every radian (57 degrees) we lean forward,            apply this amount of duty cycle.
gainGyroRate                   = 146   # For every radian/s we fall forward,                       apply this amount of duty cycle.
gainMotorAngle                 = 7     # For every radian we are ahead of the reference,           apply this amount of duty cycle
gainMotorAngularSpeed          = 12    # For every radian/s drive faster than the reference value, apply this amount of duty cycle
gainMotorAngleErrorAccumulated = 3     # For every radian x s of accumulated motor angle,          apply this amount of duty cycle

# Variables representing physical signals (more info on these in the docs)
motorAngleRaw              = 0 # The angle of "the motor", measured in raw units (degrees for the EV3). We will take the average of both motor positions as "the motor" angle, wich is essentially how far the middle of the robot has traveled.
motorAngle                 = 0 # The angle of the motor, converted to radians (2*pi radians equals 360 degrees).
motorAngleReference        = 0 # The reference angle of the motor. The robot will attempt to drive forward or backward, such that its measured position equals this reference (or close enough).
motorAngleError            = 0 # The error: the deviation of the measured motor angle from the reference. The robot attempts to make this zero, by driving toward the reference.
motorAngleErrorAccumulated = 0 # We add up all of the motor angle error in time. If this value gets out of hand, we can use it to drive the robot back to the reference position a bit quicker.
motorAngularSpeed          = 0 # The motor speed, estimated by how far the motor has turned in a given amount of time
motorAngularSpeedReference = 0 # The reference speed during manouvers: how fast we would like to drive, measured in radians per second.
motorAngularSpeedError     = 0 # The error: the deviation of the motor speed from the reference speed.
motorDutyCycle             = 0 # The 'voltage' signal we send to the motor. We calulate a new value each time, just right to keep the robot upright.
gyroRateRaw                = 0 # The raw value from the gyro sensor in rate mode.
gyroRate                   = 0 # The angular rate of the robot (how fast it is falling forward or backward), measured in radians per second.
gyroEstimatedAngle         = 0 # The gyro doesn't measure the angle of the robot, but we can estimate this angle by keeping track of the gyroRate value in time
gyroOffset                 = 0 # Over time, the gyro rate value can drift. This causes the sensor to think it is moving even when it is perfectly still. We keep track of this offset.

    
########################################################################
##
## Calibrate Gyro
##
########################################################################    
      
print("-----------------------------------")      
print("Calibrating...")

#As you hold the robot still, determine the average sensor value of 100 samples
gyroRateCalibrateCount = 100
for i in range(gyroRateCalibrateCount):
    gyroOffset = gyroOffset + gyro_sensor.value()
    time.sleep(0.01)
gyroOffset /= gyroRateCalibrateCount
       
# Print the result   
print("GyroOffset: ",gyroOffset)   
print("-----------------------------------")    
print("GO!") 
print("-----------------------------------") 

########################################################################
##
## MAIN LOOP (Press Touch Sensor to stop the program)
##
########################################################################    
    

# Remember start time because we want to set a world record
tProgramStart = time.clock()    
        
while not touchSensor.value():

    ###############################################################
    ##  Loop info
    ###############################################################
    loopCount += 1
    tLoopStart = time.clock()  

    ###############################################################
    ##
    ##  Driving and Steering. Modify this section as you like to
    ##  make your segway go anywhere!
    ##
    ###############################################################
 
    # Read e.g. your PS2 controller here. Be sure you don't drag the loop too long
    
    # Or just balance in place:
    speed    = 0 
    steering = 0

    ###############################################################
    ##  Reading the Gyro.
    ###############################################################
    gyroRateRaw = gyro_sensor.value()
    gyroRate = (gyroRateRaw - gyroOffset)*radiansPerSecondPerRawGyroUnit

    ###############################################################
    ##  Reading the Motor Position
    ###############################################################

    motorAngleRaw = (left_motor.position + right_motor.position)/2.0
    motorAngle = motorAngleRaw*radiansPerRawMotorUnit

    motorAngularSpeedReference = speed*radPerSecPerPercentSpeed
    motorAngleReference += motorAngularSpeedReference * loopTimeSec

    motorAngleError = motorAngle - motorAngleReference
    motorAngleHistory.append(motorAngle)
    
    ###############################################################
    ##  Computing Motor Speed
    ###############################################################

    motorAngularSpeed = (motorAngle - motorAngleHistory[0])/(motorAngleHistoryLength*loopTimeSec)
    motorAngularSpeedError = motorAngularSpeed - motorAngularSpeedReference

    ###############################################################
    ##  Computing the motor duty cycle value
    ###############################################################

    motorDutyCycle =(gainGyroAngle  * gyroEstimatedAngle
                   + gainGyroRate   * gyroRate
                   + gainMotorAngle * motorAngleError
                   + gainMotorAngularSpeed * motorAngularSpeedError
                   + gainMotorAngleErrorAccumulated * motorAngleErrorAccumulated)    
    
    # Clamp the value between -100 and 100
    motorDutyCycle = min(max(motorDutyCycle,-100),100)
    
    ###############################################################
    ##  Apply the signal to the motor, and add steering
    ###############################################################

    left_motor.run_forever(duty_cycle_sp=motorDutyCycle - steering)
    right_motor.run_forever(duty_cycle_sp=motorDutyCycle + steering)

    ###############################################################
    ##  Update angle estimate and Gyro Offset Estimate
    ###############################################################

    gyroEstimatedAngle += gyroRate * loopTimeSec
    gyroOffset = (1-gyroDriftCompensationRate)*gyroOffset+gyroDriftCompensationRate*gyroRateRaw

    ###############################################################
    ##  Update Accumulated Motor Error
    ###############################################################

    motorAngleErrorAccumulated += motorAngleError * loopTimeSec

    ###############################################################
    ##  Busy wait for the loop to complete
    ###############################################################
   
    while(time.clock() - tLoopStart <  loopTimeSec):
        time.sleep(0.0001) 
    
########################################################################
##
## Closing down & Cleaning up
##
######################################################################## 

# See if we have that world record
tProgramEnd = time.clock()    
    
# Turn off the motors    
for motor in motors:
    motor.stop()

# Calculate loop time
tLoop = (tProgramEnd - tProgramStart)/loopCount
print("Loop time:", tLoop*1000,"ms")

# Print a stop message
print("-----------------------------------")   
print("STOP")
print("-----------------------------------")      

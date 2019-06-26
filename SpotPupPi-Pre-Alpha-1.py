# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tlony DiCoa
# License: Public Domain
from __future__ import division
import time
from time import sleep
# Used for shutdown
import os
import board
import busio
import sys
#import adafruit_bno055
from Adafruit_BNO055 import BNO055

#Used for zip_longest
import itertools as itertools
# used for rounding up
import math
from statistics import median
# so we can run more than 1 loop at the same time need to decide multiproc or threading
#import threading
#UltraSonic Sensor
from gpiozero import DistanceSensor

# Import the PCA9685 module.
import board
import busio
import adafruit_pca9685
from adafruit_servokit import ServoKit
servo = ServoKit(channels=16)
#i2c = busio.I2C(board.SCL, board.SDA)
#servo = adafruit_pca9685.PCA9685(i2c)

# Import GPIO 
import RPi.GPIO as GPIO


# Import Speech Recognition
import speech_recognition as sr

# Setup GPIO allocate Pins

SW1 = 5
LED1 = 12
LED2 = 13
US_RIGHT_TRIG = 16
US_RIGHT_ECHO = 19
US_LEFT_TRIG = 20
US_LEFT_ECHO = 21
orientation_rst = 4
RELAY1 = 17
RELAY2 = 18

orientation = BNO055.BNO055(serial_port='/dev/serial0', rst=orientation_rst)
# GPIO SETUP Direction
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(SW1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(SW1,GPIO.FALLING)
GPIO.setup(LED1,GPIO.OUT)
GPIO.setup(LED2,GPIO.OUT)
GPIO.setup(orientation_rst,GPIO.OUT)
GPIO.setup(US_RIGHT_TRIG, GPIO.OUT)
GPIO.setup(US_RIGHT_ECHO, GPIO.IN)
GPIO.setup(US_LEFT_TRIG, GPIO.OUT)
GPIO.setup(US_LEFT_ECHO, GPIO.IN)
GPIO.setup(RELAY1,GPIO.OUT)
GPIO.setup(RELAY2,GPIO.OUT)

ultrasonicright = DistanceSensor(echo=US_RIGHT_ECHO, trigger=US_RIGHT_TRIG, max_distance=3)
ultrasonicleft = DistanceSensor(echo=US_LEFT_ECHO, trigger=US_LEFT_TRIG, max_distance=3)

# Uncomment to enable debug output.
# import logging
# logging.basicConfig(level=logging.DEBUG)

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo settings
servo_min = 5  
servo_max = 175

standing_leg_1 = 90
standing_leg_2 =75
standing_leg_3 = 120

lie_speed_increment = 3
lie_leg_1 = 90
lie_leg_2 = 90
lie_leg_3 = 175

sit_speed_increment = 1
sit_fleg_1 = 100
sit_fleg_2 = 170
sit_fleg_3 = 30
sit_rleg_1 = 135
sit_rleg_2 = 40
sit_rleg_3 = 170

#Walk curve's and speed
walk_speed_delay = 0.00
walk_speed_increment = 10
walk_stride = 0
lean_forward_2 = 70
lean_forward_3 = 125
lean_adjust_2 = 0
lean_adjust_3 = 0

#Lift leg
walk_leg_2_position_1 = lean_forward_2 
walk_leg_3_position_1 = lean_forward_3

walk_leg_2_position_2 = 40 + 10
walk_leg_3_position_2 = 135 - 10

#Reach forward
walk_leg_2_position_3 = 40 + 10
walk_leg_3_position_3 = 175 - 10

walk_leg_2_position_4 = 100 - 10
walk_leg_3_position_4 = 115 

#Leg drive
walk_leg_2_position_5 = 95 - 10
walk_leg_3_position_5 = 125

walk_leg_2_position_6 = 90 - 8 
walk_leg_3_position_6 = 125

walk_leg_2_position_7 = 85 - 7
walk_leg_3_position_7 = 125

walk_leg_2_position_8 = 83 - 6
walk_leg_3_position_8 = 125

walk_leg_2_position_9 = 80 - 5
walk_leg_3_position_9 = 125

walk_leg_2_position_10 = 75 - 2
walk_leg_3_position_10 = 120



#Balancing 
accuracy = 10 # We cant use float numbers so mulptiply orintation data by 1, 10, 100, 1000 (1 least sensitive)
pitch_reaction = 1.7 # 0.0-4.0  1.6 seems the best value note servo value should not exceed 180 (Look to cap this)
roll_reaction = 1.0
roll_adjust = 0 #alter senor output
pitch_adjust = -5 #alter senor output

# Set servo assignment
left_front_leg_1 = 0
left_front_leg_2 = 1
left_front_leg_3 = 2

right_front_leg_1 = 3
right_front_leg_2 = 4
right_front_leg_3 = 5

right_rear_leg_1 = 6
right_rear_leg_2 = 7
right_rear_leg_3 = 8

left_rear_leg_1 = 9
left_rear_leg_2 = 10
left_rear_leg_3 = 11

# Set Servo Adjust (Setuo with reaction set to zero)
left_front_leg_1_adjust = 0
left_front_leg_2_adjust = -5
left_front_leg_3_adjust = 0

right_front_leg_1_adjust = 0
right_front_leg_2_adjust = -10
right_front_leg_3_adjust = -10

right_rear_leg_1_adjust = 5
right_rear_leg_2_adjust = 0
right_rear_leg_3_adjust = 0

left_rear_leg_1_adjust = 10
left_rear_leg_2_adjust = 10
left_rear_leg_3_adjust = 0

debug = 'on' # on/off
puppy_name = 'Spot'

def flashLED(speed, time):
        for x in range(0, time):
                GPIO.output(LED1, GPIO.LOW)
                sleep(speed)
                GPIO.output(LED1, GPIO.HIGH)
                sleep(speed)
 
  
def QuadPress():
      print ("SW 1 Pressed Four Times So! Blink LED FAST!")
      flashLED(0.05, 100)

                
def triplePress():
      print ("SW 1 Pressed Three times So! Turn OFF Servos")
      GPIO.output(RELAY1,GPIO.HIGH)
      GPIO.output(LED2,GPIO.LOW)

def doublePress():
        print('Turn on servos')
        # Turn on servos.
        GPIO.output(LED2,GPIO.HIGH)
        GPIO.output(RELAY1,GPIO.LOW)
        

def singlePress():
      print('SW 1 Single Press - Lets Walk, hold button to stop')
      StandUp()

def Walk2():
 step_count = 0
 cycle_count = 0
 print('starting walk')
 while True:

         # Remove data spikes by selecting the middle value and round up
         heading, roll, pitch = orientation.read_euler()
         # Get a vaule to feed into servo reation to assist with balance
         print('Step Count:', step_count, 'cycle_count:',cycle_count)
         if debug == 'on':
          print('Roll', roll)  
         if roll in range(accuracy*50,accuracy*500):
          print ('oops I fell over+', roll)
          GetUp1()
          
         if roll in range(accuracy*-500,accuracy*-50):
          print ('oops I fell over-', roll)
          GetUp2()
              
         roll_steady_value = (math.ceil(roll_reaction*(roll+roll_adjust)))
         pitch_steady_value = (math.ceil(pitch_reaction*(pitch+pitch_adjust)))
         
         if debug == 'on':
           print('Steady Roll', roll_steady_value, 'Steady Pitch', pitch_steady_value)
           print('Spots first step (Front Right Leg)')
           
         if cycle_count == 0:
          walk_right_front_leg_2 = walk_leg_2_position_7
          walk_right_front_leg_3 = walk_leg_3_position_7
          walk_right_rear_leg_2 = walk_leg_2_position_10
          walk_right_rear_leg_3 = walk_leg_3_position_10
          walk_left_rear_leg_2 = walk_leg_2_position_5
          walk_left_rear_leg_3 = walk_leg_3_position_5
          walk_left_front_leg_2= walk_leg_2_position_2
          walk_left_front_leg_3 = walk_leg_3_position_2
          
          if step_count < 2:
           print('leaning forward')
           servo.servo[right_front_leg_1].angle = min(servo_max,max(servo_min, right_front_leg_1_adjust + standing_leg_1 - roll_steady_value))
           servo.servo[right_front_leg_2].angle = min(servo_max,max(servo_min, right_front_leg_2_adjust + right_front_leg_2_adjust + lean_forward_2))
           servo.servo[right_front_leg_3].angle = min(servo_max,max(servo_min, right_front_leg_3_adjust + lean_forward_3 - roll_steady_value + pitch_steady_value))
           servo.servo[left_front_leg_1].angle = min(servo_max,max(servo_min, 180 - left_front_leg_1_adjust - standing_leg_1 - roll_steady_value))
           servo.servo[left_front_leg_2].angle = min(servo_max,max(servo_min, 180 - left_front_leg_2_adjust - lean_forward_2))
           servo.servo[left_front_leg_3].angle = min(servo_max,max(servo_min, 180 - left_front_leg_3_adjust - lean_forward_3 - roll_steady_value - pitch_steady_value))
           servo.servo[right_rear_leg_1].angle = min(servo_max,max(servo_min, 180 - right_rear_leg_1_adjust - standing_leg_1 + roll_steady_value))
           servo.servo[right_rear_leg_2].angle = min(servo_max,max(servo_min, right_rear_leg_2_adjust + lean_forward_2))
           servo.servo[right_rear_leg_3].angle = min(servo_max,max(servo_min, right_rear_leg_3_adjust + lean_forward_3+  roll_steady_value - pitch_steady_value))
           servo.servo[left_rear_leg_1].angle = min(servo_max,max(servo_min, left_rear_leg_1_adjust + standing_leg_1 + roll_steady_value))
           servo.servo[left_rear_leg_2].angle = min(servo_max,max(servo_min, 180 - left_rear_leg_2_adjust - lean_forward_2))
           servo.servo[left_rear_leg_3].angle = min(servo_max,max(servo_min, 180 - left_rear_leg_3_adjust - lean_forward_3-roll_steady_value + pitch_steady_value))
          
         if cycle_count == 1:
          walk_right_front_leg_2 = walk_leg_2_position_9
          walk_right_front_leg_3 = walk_leg_3_position_9
          walk_right_rear_leg_2 = walk_leg_2_position_1
          walk_right_rear_leg_3 = walk_leg_3_position_1
          walk_left_rear_leg_2 = walk_leg_2_position_7
          walk_left_rear_leg_3 = walk_leg_3_position_7
          walk_left_front_leg_2= walk_leg_2_position_3
          walk_left_front_leg_3 = walk_leg_3_position_3
          
         if cycle_count == 2:
          walk_right_front_leg_2 = walk_leg_2_position_10
          walk_right_front_leg_3 = walk_leg_3_position_10
          walk_right_rear_leg_2 = walk_leg_2_position_2
          walk_right_rear_leg_3 = walk_leg_3_position_2
          walk_left_rear_leg_2 = walk_leg_2_position_9
          walk_left_rear_leg_3 = walk_leg_3_position_9
          walk_left_front_leg_2= walk_leg_2_position_4
          walk_left_front_leg_3 = walk_leg_3_position_4
         
         if cycle_count == 3:
          walk_right_front_leg_2 = walk_leg_2_position_1
          walk_right_front_leg_3 = walk_leg_3_position_1
          walk_right_rear_leg_2 = walk_leg_2_position_3
          walk_right_rear_leg_3 = walk_leg_3_position_3
          walk_left_rear_leg_2 = walk_leg_2_position_10
          walk_left_rear_leg_3 = walk_leg_3_position_10
          walk_left_front_leg_2= walk_leg_2_position_5
          walk_left_front_leg_3 = walk_leg_3_position_5
          
         if cycle_count == 4:
          walk_right_front_leg_2 = walk_leg_2_position_2
          walk_right_front_leg_3 = walk_leg_3_position_2
          walk_right_rear_leg_2 = walk_leg_2_position_4
          walk_right_rear_leg_3 = walk_leg_3_position_4
          walk_left_rear_leg_2 = walk_leg_2_position_1
          walk_left_rear_leg_3 = walk_leg_3_position_1
          walk_left_front_leg_2= walk_leg_2_position_7
          walk_left_front_leg_3 = walk_leg_3_position_7
          
         if cycle_count == 5:
          walk_right_front_leg_2 = walk_leg_2_position_3
          walk_right_front_leg_3 = walk_leg_3_position_3
          walk_right_rear_leg_2 = walk_leg_2_position_5
          walk_right_rear_leg_3 = walk_leg_3_position_5
          walk_left_rear_leg_2 = walk_leg_2_position_2
          walk_left_rear_leg_3 = walk_leg_3_position_2
          walk_left_front_leg_2= walk_leg_2_position_9
          walk_left_front_leg_3 = walk_leg_3_position_9
          
         if cycle_count == 6:
          walk_right_front_leg_2 = walk_leg_2_position_4
          walk_right_front_leg_3 = walk_leg_3_position_4
          walk_right_rear_leg_2 = walk_leg_2_position_7
          walk_right_rear_leg_3 = walk_leg_3_position_7
          walk_left_rear_leg_2 = walk_leg_2_position_3
          walk_left_rear_leg_3 = walk_leg_3_position_3
          walk_left_front_leg_2= walk_leg_2_position_10
          walk_left_front_leg_3 = walk_leg_3_position_10
          
         if cycle_count == 7:
          walk_right_front_leg_2 = walk_leg_2_position_5
          walk_right_front_leg_3 = walk_leg_3_position_5
          walk_right_rear_leg_2 = walk_leg_2_position_9
          walk_right_rear_leg_3 = walk_leg_3_position_9
          walk_left_rear_leg_2 = walk_leg_2_position_4
          walk_left_rear_leg_3 = walk_leg_3_position_4
          walk_left_front_leg_2= walk_leg_2_position_1
          walk_left_front_leg_3 = walk_leg_3_position_1
          
    
         
         if servo.servo[right_front_leg_1].angle < (right_front_leg_1_adjust + standing_leg_1 - roll_steady_value):
             rfleg1 = range(math.ceil(servo.servo[right_front_leg_1].angle), min(servo_max,max(servo_min, right_front_leg_1_adjust + standing_leg_1 - roll_steady_value)), walk_speed_increment)
         else:
             rfleg1 = range(math.ceil(servo.servo[right_front_leg_1].angle), min(servo_max,max(servo_min, right_front_leg_1_adjust + standing_leg_1 - roll_steady_value)), -walk_speed_increment)   
         if servo.servo[right_front_leg_2].angle < (right_front_leg_2_adjust + walk_right_front_leg_2):
             rfleg2 = range(math.ceil(servo.servo[right_front_leg_2].angle), min(servo_max,max(servo_min, right_front_leg_2_adjust + walk_right_front_leg_2 - lean_adjust_2)), walk_speed_increment)
         else:
             rfleg2 = range(math.ceil(servo.servo[right_front_leg_2].angle), min(servo_max,max(servo_min, right_front_leg_2_adjust + walk_right_front_leg_2 - lean_adjust_2)), -walk_speed_increment)  
         if servo.servo[right_front_leg_3].angle < (right_front_leg_3_adjust + walk_right_front_leg_3 + roll_steady_value + pitch_steady_value):   
             rfleg3 = range(math.ceil(servo.servo[right_front_leg_3].angle), min(servo_max,max(servo_min, right_front_leg_3_adjust + walk_right_front_leg_3 + roll_steady_value + pitch_steady_value + lean_adjust_3)), walk_speed_increment)
         else:
             rfleg3 = range(math.ceil(servo.servo[right_front_leg_3].angle), min(servo_max,max(servo_min, right_front_leg_3_adjust + walk_right_front_leg_3 + roll_steady_value + pitch_steady_value + lean_adjust_3)), -walk_speed_increment)      
         if servo.servo[left_front_leg_1].angle < (180 - left_front_leg_1_adjust - standing_leg_1 - roll_steady_value):
             lfleg1 = range(math.ceil(servo.servo[left_front_leg_1].angle), min(servo_max,max(servo_min, 180 - left_front_leg_1_adjust - standing_leg_1 - roll_steady_value)), walk_speed_increment)
         else:
             lfleg1 = range(math.ceil(servo.servo[left_front_leg_1].angle), min(servo_max,max(servo_min, 180 - left_front_leg_1_adjust - standing_leg_1 - roll_steady_value)), -walk_speed_increment)
         if servo.servo[left_front_leg_2].angle < (180 - left_front_leg_2_adjust - walk_left_front_leg_2):
             lfleg2 = range(math.ceil(servo.servo[left_front_leg_2].angle), min(servo_max,max(servo_min, 180 - left_front_leg_2_adjust - walk_left_front_leg_2 + lean_adjust_2)), walk_speed_increment)
         else:
             lfleg2 = range(math.ceil(servo.servo[left_front_leg_2].angle), min(servo_max,max(servo_min, 180 - left_front_leg_2_adjust - walk_left_front_leg_2 + lean_adjust_2)), -walk_speed_increment)
         if servo.servo[left_front_leg_3].angle < (180 - left_front_leg_3_adjust - walk_left_front_leg_3 - roll_steady_value - pitch_steady_value):
             lfleg3 = range(math.ceil(servo.servo[left_front_leg_3].angle), min(servo_max,max(servo_min, 180 - left_front_leg_3_adjust - walk_left_front_leg_3 - roll_steady_value - pitch_steady_value - lean_adjust_3)), walk_speed_increment)
         else:
             lfleg3 = range(math.ceil(servo.servo[left_front_leg_3].angle), min(servo_max,max(servo_min, 180 - left_front_leg_3_adjust - walk_left_front_leg_3 - roll_steady_value - pitch_steady_value - lean_adjust_3)), -walk_speed_increment)
         if servo.servo[right_rear_leg_1].angle < (180 - right_rear_leg_1_adjust - standing_leg_1 + roll_steady_value):
             rrleg1 = range(math.ceil(servo.servo[right_rear_leg_1].angle), min(servo_max,max(servo_min, 180 - right_rear_leg_1_adjust - standing_leg_1 + roll_steady_value)), walk_speed_increment)
         else:
             rrleg1 = range(math.ceil(servo.servo[right_rear_leg_1].angle), min(servo_max,max(servo_min, 180 - right_rear_leg_1_adjust - standing_leg_1 + roll_steady_value)), -walk_speed_increment)
         if servo.servo[right_rear_leg_2].angle < (right_rear_leg_2_adjust + walk_right_rear_leg_2):
             rrleg2 = range(math.ceil(servo.servo[right_rear_leg_2].angle), min(servo_max,max(servo_min, right_rear_leg_2_adjust + walk_right_rear_leg_2)), walk_speed_increment)
         else:
             rrleg2 = range(math.ceil(servo.servo[right_rear_leg_2].angle), min(servo_max,max(servo_min, right_rear_leg_2_adjust + walk_right_rear_leg_2)), -walk_speed_increment)
         if servo.servo[right_rear_leg_3].angle < (right_rear_leg_3_adjust + walk_right_rear_leg_3 + roll_steady_value - pitch_steady_value):
             rrleg3 = range(math.ceil(servo.servo[right_rear_leg_3].angle), min(servo_max,max(servo_min, right_rear_leg_3_adjust + walk_right_rear_leg_3 + roll_steady_value - pitch_steady_value)), walk_speed_increment)
         else:
             rrleg3 = range(math.ceil(servo.servo[right_rear_leg_3].angle), min(servo_max,max(servo_min, right_rear_leg_3_adjust + walk_right_rear_leg_3 + roll_steady_value - pitch_steady_value)), -walk_speed_increment)
         if servo.servo[left_rear_leg_1].angle < (left_rear_leg_1_adjust + standing_leg_1 + roll_steady_value):
             lrleg1 = range(math.ceil(servo.servo[left_rear_leg_1].angle), min(servo_max,max(servo_min, left_rear_leg_1_adjust + standing_leg_1 + roll_steady_value)), walk_speed_increment)
         else:
             lrleg1 = range(math.ceil(servo.servo[left_rear_leg_1].angle), min(servo_max,max(servo_min, left_rear_leg_1_adjust + standing_leg_1 + roll_steady_value)), -walk_speed_increment)
         if servo.servo[left_rear_leg_2].angle < (180 - left_rear_leg_2_adjust - walk_left_rear_leg_2):
             lrleg2 = range(math.ceil(servo.servo[left_rear_leg_2].angle), min(servo_max,max(servo_min, 180 - left_rear_leg_2_adjust - walk_left_rear_leg_2)), walk_speed_increment)
         else:
             lrleg2 = range(math.ceil(servo.servo[left_rear_leg_2].angle), min(servo_max,max(servo_min, 180 - left_rear_leg_2_adjust - walk_left_rear_leg_2)), -walk_speed_increment)
         if servo.servo[left_rear_leg_3].angle < (180 - left_rear_leg_3_adjust - walk_left_rear_leg_3 - roll_steady_value + pitch_steady_value):
             lrleg3 = range(math.ceil(servo.servo[left_rear_leg_3].angle), min(servo_max,max(servo_min, 180 - left_rear_leg_3_adjust - walk_left_rear_leg_3 - roll_steady_value + pitch_steady_value)), walk_speed_increment)
         else:
             lrleg3 = range(math.ceil(servo.servo[left_rear_leg_3].angle), min(servo_max,max(servo_min, 180 - left_rear_leg_3_adjust - walk_left_rear_leg_3 - roll_steady_value + pitch_steady_value)), -walk_speed_increment)
         if debug == 'on':    
             print('rfleg1', rfleg1, 'rfleg2', rfleg2, 'rfleg3', rfleg3, 'lfleg1', lfleg1, 'lfleg2', lfleg2, 'lfleg3', lfleg3, 'rrleg1', rrleg1, 'rrleg2', rrleg2, 'rrleg3', rrleg3, 'lrleg1', lrleg1, 'lrleg2', lrleg2, 'lrleg3', lrleg3)
         for (a, b, c, d, e, f, g, h, i, j, k, l) in itertools.zip_longest(rfleg1, rfleg2, rfleg3, lfleg1, lfleg2, lfleg3, rrleg1, rrleg2, rrleg3, lrleg1, lrleg2, lrleg3,fillvalue='finished'):
             if a == 'finished':
                 servo.servo[right_front_leg_1].angle = min(servo_max,max(servo_min, right_front_leg_1_adjust + standing_leg_1 - roll_steady_value))
             else:
                 servo.servo[right_front_leg_1].angle = a
             if b == 'finished':
                 servo.servo[right_front_leg_2].angle = min(servo_max,max(servo_min, right_front_leg_2_adjust + walk_right_front_leg_2 - lean_adjust_2)) 
             else:
                 servo.servo[right_front_leg_2].angle = b
             if c == 'finished':
                 servo.servo[right_front_leg_3].angle = min(servo_max,max(servo_min, right_front_leg_3_adjust + walk_right_front_leg_3 + roll_steady_value + pitch_steady_value + lean_adjust_3))
             else:
                 servo.servo[right_front_leg_3].angle = c
             if d == 'finished':
                 servo.servo[left_front_leg_1].angle = min(servo_max,max(servo_min, 180 - left_front_leg_1_adjust - standing_leg_1 - roll_steady_value))
             else:
                 servo.servo[left_front_leg_1].angle = d
             if e == 'finished':
                 servo.servo[left_front_leg_2].angle = min(servo_max,max(servo_min, 180 - left_front_leg_2_adjust - walk_left_front_leg_2 + lean_adjust_2))
             else:
                 servo.servo[left_front_leg_2].angle = e
             if f == 'finished':
                 servo.servo[left_front_leg_3].angle = min(servo_max,max(servo_min, 180 - left_front_leg_3_adjust - walk_left_front_leg_3 - roll_steady_value - pitch_steady_value - lean_adjust_3))
             else:
                 servo.servo[left_front_leg_3].angle = f
             if g == 'finished':
                 servo.servo[right_rear_leg_1].angle = min(servo_max,max(servo_min, 180 - right_rear_leg_1_adjust - standing_leg_1 + roll_steady_value))
             else:
                 servo.servo[right_rear_leg_1].angle = g
             if h == 'finished':
                 servo.servo[right_rear_leg_2].angle = min(servo_max,max(servo_min, right_rear_leg_2_adjust + walk_right_rear_leg_2))
             else:
                 servo.servo[right_rear_leg_2].angle = h
             if i == 'finished':
                 servo.servo[right_rear_leg_3].angle = min(servo_max,max(servo_min, right_rear_leg_3_adjust + walk_right_rear_leg_3 + roll_steady_value - pitch_steady_value))
             else:
                 servo.servo[right_rear_leg_3].angle = i
             if j == 'finished':
                 servo.servo[left_rear_leg_1].angle = min(servo_max,max(servo_min, left_rear_leg_1_adjust + standing_leg_1 + roll_steady_value))
             else:
                 servo.servo[left_rear_leg_1].angle = j
             if k == 'finished':
                 servo.servo[left_rear_leg_2].angle = min(servo_max,max(servo_min, 180 - left_rear_leg_2_adjust - walk_left_rear_leg_2))
             else:
                 servo.servo[left_rear_leg_2].angle = k
             if l == 'finished':
                 servo.servo[left_rear_leg_3].angle = min(servo_max,max(servo_min, 180 - left_rear_leg_3_adjust - walk_left_rear_leg_3 - roll_steady_value + pitch_steady_value))
             else:
                 servo.servo[left_rear_leg_3].angle = l
             if debug == 'on':
                 print(
              #         servo.servo[right_front_leg_1].angle,
              #         servo.servo[right_front_leg_2].angle,
              #         servo.servo[right_front_leg_3].angle,
              #         servo.servo[left_front_leg_1].angle,
              #         servo.servo[left_front_leg_2].angle,
              #         servo.servo[left_front_leg_3].angle,
                       servo.servo[right_rear_leg_1].angle,
                       servo.servo[right_rear_leg_2].angle,
                       servo.servo[right_rear_leg_3].angle,
              #         servo.servo[left_rear_leg_1].angle,
              #         servo.servo[left_rear_leg_2].angle,
              #        servo.servo[left_rear_leg_3].angle,
                       'cycle count', cycle_count)
             time.sleep(walk_speed_delay)
         cycle_count = cycle_count+1
         print('cycle count', cycle_count)
         if cycle_count > 7:
             cycle_count = 0
             step_count = step_count+2
         
         dist = math.ceil(ultrasonicright.distance*100)
         if debug == 'on':
             print('distance', dist)
         if dist in range (1, 15):
            print('WALL! Stop!', ultrasonicright.distance)
            break
         input_state = GPIO.input(SW1)
         if input_state == False:
            print ('Single press stopping')
            break  
         else:
            GPIO.output(LED1,GPIO.LOW)
         BackGroundTasks()
         
def Walk():
 step_count = 0
 cycle_count = 0
 print('starting walk')
 while True:

         # Remove data spikes by selecting the middle value and round up
         heading, roll, pitch = orientation.read_euler()
         # Get a vaule to feed into servo reation to assist with balance
         print('Step Count:', step_count, 'cycle_count:',cycle_count)
         if debug == 'on':
          print('Roll', roll)  
         if roll in range(accuracy*50,accuracy*500):
          print ('oops I fell over+', roll)
          GetUp1()
          
         if roll in range(accuracy*-500,accuracy*-50):
          print ('oops I fell over-', roll)
          GetUp2()
              
         roll_steady_value = (math.ceil(roll_reaction*(roll+roll_adjust)))
         pitch_steady_value = (math.ceil(pitch_reaction*(pitch+pitch_adjust)))
         
         if debug == 'on':
           print('Steady Roll', roll_steady_value, 'Steady Pitch', pitch_steady_value)
           print('Spots first step (Front Right Leg)')
           
         if cycle_count == 0:
          walk_leg_2_cycle_1 = walk_leg_2_position_1
          walk_leg_3_cycle_1 = walk_leg_3_position_1
          walk_leg_2_cycle_2 = walk_leg_2_position_6
          walk_leg_3_cycle_2 = walk_leg_3_position_6
          if step_count < 2:
           print('leaning forward')
           servo.servo[right_front_leg_1].angle = min(servo_max,max(servo_min, right_front_leg_1_adjust + standing_leg_1 - roll_steady_value))
           servo.servo[right_front_leg_2].angle = min(servo_max,max(servo_min, right_front_leg_2_adjust + right_front_leg_2_adjust + lean_forward_2))
           servo.servo[right_front_leg_3].angle = min(servo_max,max(servo_min, right_front_leg_3_adjust + lean_forward_3 - roll_steady_value + pitch_steady_value))
           servo.servo[left_front_leg_1].angle = min(servo_max,max(servo_min, 180 - left_front_leg_1_adjust - standing_leg_1 - roll_steady_value))
           servo.servo[left_front_leg_2].angle = min(servo_max,max(servo_min, 180 - left_front_leg_2_adjust - lean_forward_2))
           servo.servo[left_front_leg_3].angle = min(servo_max,max(servo_min, 180 - left_front_leg_3_adjust - lean_forward_3 - roll_steady_value - pitch_steady_value))
           servo.servo[right_rear_leg_1].angle = min(servo_max,max(servo_min, 180 - right_rear_leg_1_adjust - standing_leg_1 + roll_steady_value))
           servo.servo[right_rear_leg_2].angle = min(servo_max,max(servo_min, right_rear_leg_2_adjust + lean_forward_2))
           servo.servo[right_rear_leg_3].angle = min(servo_max,max(servo_min, right_rear_leg_3_adjust + lean_forward_3+  roll_steady_value - pitch_steady_value))
           servo.servo[left_rear_leg_1].angle = min(servo_max,max(servo_min, left_rear_leg_1_adjust + standing_leg_1 + roll_steady_value))
           servo.servo[left_rear_leg_2].angle = min(servo_max,max(servo_min, 180 - left_rear_leg_2_adjust - lean_forward_2))
           servo.servo[left_rear_leg_3].angle = min(servo_max,max(servo_min, 180 - left_rear_leg_3_adjust - lean_forward_3-roll_steady_value + pitch_steady_value))
          
         if cycle_count == 1:
          walk_leg_2_cycle_1 = walk_leg_2_position_2
          walk_leg_3_cycle_1 = walk_leg_3_position_2
          walk_leg_2_cycle_2 = walk_leg_2_position_7
          walk_leg_3_cycle_2 = walk_leg_3_position_7
          
         if cycle_count == 2:
          walk_leg_2_cycle_1 = walk_leg_2_position_3
          walk_leg_3_cycle_1 = walk_leg_3_position_3
          walk_leg_2_cycle_2 = walk_leg_2_position_8
          walk_leg_3_cycle_2 = walk_leg_3_position_8
         
         if cycle_count == 3:
          walk_leg_2_cycle_1 = walk_leg_2_position_4
          walk_leg_3_cycle_1 = walk_leg_3_position_4
          walk_leg_2_cycle_2 = walk_leg_2_position_9
          walk_leg_3_cycle_2 = walk_leg_3_position_9
          
         if cycle_count == 4:
          walk_leg_2_cycle_1 = walk_leg_2_position_5
          walk_leg_3_cycle_1 = walk_leg_3_position_5
          walk_leg_2_cycle_2 = walk_leg_2_position_10
          walk_leg_3_cycle_2 = walk_leg_3_position_10
          
         if cycle_count == 5:
          walk_leg_2_cycle_1 = walk_leg_2_position_6
          walk_leg_3_cycle_1 = walk_leg_3_position_6
          walk_leg_2_cycle_2 = walk_leg_2_position_1
          walk_leg_3_cycle_2 = walk_leg_3_position_1
          
         if cycle_count == 6:
          walk_leg_2_cycle_1 = walk_leg_2_position_7
          walk_leg_3_cycle_1 = walk_leg_3_position_7
          walk_leg_2_cycle_2 = walk_leg_2_position_2
          walk_leg_3_cycle_2 = walk_leg_3_position_2
          
         if cycle_count == 7:
          walk_leg_2_cycle_1 = walk_leg_2_position_8
          walk_leg_3_cycle_1 = walk_leg_3_position_8
          walk_leg_2_cycle_2 = walk_leg_2_position_3
          walk_leg_3_cycle_2 = walk_leg_3_position_3
          
         if cycle_count == 8:
          walk_leg_2_cycle_1 = walk_leg_2_position_9
          walk_leg_3_cycle_1 = walk_leg_3_position_9
          walk_leg_2_cycle_2 = walk_leg_2_position_4
          walk_leg_3_cycle_2 = walk_leg_3_position_4
         
         if cycle_count == 9:
          walk_leg_2_cycle_1 = walk_leg_2_position_10
          walk_leg_3_cycle_1 = walk_leg_3_position_10
          walk_leg_2_cycle_2 = walk_leg_2_position_5
          walk_leg_3_cycle_2 = walk_leg_3_position_5         
    
         
         if servo.servo[right_front_leg_1].angle < (right_front_leg_1_adjust + standing_leg_1 - roll_steady_value):
             rfleg1 = range(math.ceil(servo.servo[right_front_leg_1].angle), min(servo_max,max(servo_min, right_front_leg_1_adjust + standing_leg_1 - roll_steady_value)), walk_speed_increment)
         else:
             rfleg1 = range(math.ceil(servo.servo[right_front_leg_1].angle), min(servo_max,max(servo_min, right_front_leg_1_adjust + standing_leg_1 - roll_steady_value)), -walk_speed_increment)   
         if servo.servo[right_front_leg_2].angle < (right_front_leg_2_adjust + walk_leg_2_cycle_1):
             rfleg2 = range(math.ceil(servo.servo[right_front_leg_2].angle), min(servo_max,max(servo_min, right_front_leg_2_adjust + walk_leg_2_cycle_1 - lean_adjust_2)), walk_speed_increment)
         else:
             rfleg2 = range(math.ceil(servo.servo[right_front_leg_2].angle), min(servo_max,max(servo_min, right_front_leg_2_adjust + walk_leg_2_cycle_1 - lean_adjust_2)), -walk_speed_increment)  
         if servo.servo[right_front_leg_3].angle < (right_front_leg_3_adjust + walk_leg_3_cycle_1 + roll_steady_value + pitch_steady_value):   
             rfleg3 = range(math.ceil(servo.servo[right_front_leg_3].angle), min(servo_max,max(servo_min, right_front_leg_3_adjust + walk_leg_3_cycle_1 + roll_steady_value + pitch_steady_value + lean_adjust_3)), walk_speed_increment)
         else:
             rfleg3 = range(math.ceil(servo.servo[right_front_leg_3].angle), min(servo_max,max(servo_min, right_front_leg_3_adjust + walk_leg_3_cycle_1 + roll_steady_value + pitch_steady_value + lean_adjust_3)), -walk_speed_increment)      
         if servo.servo[left_front_leg_1].angle < (180 - left_front_leg_1_adjust - standing_leg_1 - roll_steady_value):
             lfleg1 = range(math.ceil(servo.servo[left_front_leg_1].angle), min(servo_max,max(servo_min, 180 - left_front_leg_1_adjust - standing_leg_1 - roll_steady_value)), walk_speed_increment)
         else:
             lfleg1 = range(math.ceil(servo.servo[left_front_leg_1].angle), min(servo_max,max(servo_min, 180 - left_front_leg_1_adjust - standing_leg_1 - roll_steady_value)), -walk_speed_increment)
         if servo.servo[left_front_leg_2].angle < (180 - left_front_leg_2_adjust - walk_leg_2_cycle_2):
             lfleg2 = range(math.ceil(servo.servo[left_front_leg_2].angle), min(servo_max,max(servo_min, 180 - left_front_leg_2_adjust - walk_leg_2_cycle_2 + lean_adjust_2)), walk_speed_increment)
         else:
             lfleg2 = range(math.ceil(servo.servo[left_front_leg_2].angle), min(servo_max,max(servo_min, 180 - left_front_leg_2_adjust - walk_leg_2_cycle_2 + lean_adjust_2)), -walk_speed_increment)
         if servo.servo[left_front_leg_3].angle < (180 - left_front_leg_3_adjust - walk_leg_3_cycle_2 - roll_steady_value - pitch_steady_value):
             lfleg3 = range(math.ceil(servo.servo[left_front_leg_3].angle), min(servo_max,max(servo_min, 180 - left_front_leg_3_adjust - walk_leg_3_cycle_2 - roll_steady_value - pitch_steady_value - lean_adjust_3)), walk_speed_increment)
         else:
             lfleg3 = range(math.ceil(servo.servo[left_front_leg_3].angle), min(servo_max,max(servo_min, 180 - left_front_leg_3_adjust - walk_leg_3_cycle_2 - roll_steady_value - pitch_steady_value - lean_adjust_3)), -walk_speed_increment)
         if servo.servo[right_rear_leg_1].angle < (180 - right_rear_leg_1_adjust - standing_leg_1 + roll_steady_value):
             rrleg1 = range(math.ceil(servo.servo[right_rear_leg_1].angle), min(servo_max,max(servo_min, 180 - right_rear_leg_1_adjust - standing_leg_1 + roll_steady_value)), walk_speed_increment)
         else:
             rrleg1 = range(math.ceil(servo.servo[right_rear_leg_1].angle), min(servo_max,max(servo_min, 180 - right_rear_leg_1_adjust - standing_leg_1 + roll_steady_value)), -walk_speed_increment)
         if servo.servo[right_rear_leg_2].angle < (right_rear_leg_2_adjust + walk_leg_2_cycle_2):
             rrleg2 = range(math.ceil(servo.servo[right_rear_leg_2].angle), min(servo_max,max(servo_min, right_rear_leg_2_adjust + walk_leg_2_cycle_2)), walk_speed_increment)
         else:
             rrleg2 = range(math.ceil(servo.servo[right_rear_leg_2].angle), min(servo_max,max(servo_min, right_rear_leg_2_adjust + walk_leg_2_cycle_2)), -walk_speed_increment)
         if servo.servo[right_rear_leg_3].angle < (right_rear_leg_3_adjust + walk_leg_3_cycle_2 + roll_steady_value - pitch_steady_value):
             rrleg3 = range(math.ceil(servo.servo[right_rear_leg_3].angle), min(servo_max,max(servo_min, right_rear_leg_3_adjust + walk_leg_3_cycle_2 + roll_steady_value - pitch_steady_value)), walk_speed_increment)
         else:
             rrleg3 = range(math.ceil(servo.servo[right_rear_leg_3].angle), min(servo_max,max(servo_min, right_rear_leg_3_adjust + walk_leg_3_cycle_2 + roll_steady_value - pitch_steady_value)), -walk_speed_increment)
         if servo.servo[left_rear_leg_1].angle < (left_rear_leg_1_adjust + standing_leg_1 + roll_steady_value):
             lrleg1 = range(math.ceil(servo.servo[left_rear_leg_1].angle), min(servo_max,max(servo_min, left_rear_leg_1_adjust + standing_leg_1 + roll_steady_value)), walk_speed_increment)
         else:
             lrleg1 = range(math.ceil(servo.servo[left_rear_leg_1].angle), min(servo_max,max(servo_min, left_rear_leg_1_adjust + standing_leg_1 + roll_steady_value)), -walk_speed_increment)
         if servo.servo[left_rear_leg_2].angle < (180 - left_rear_leg_2_adjust - walk_leg_2_cycle_1):
             lrleg2 = range(math.ceil(servo.servo[left_rear_leg_2].angle), min(servo_max,max(servo_min, 180 - left_rear_leg_2_adjust - walk_leg_2_cycle_1)), walk_speed_increment)
         else:
             lrleg2 = range(math.ceil(servo.servo[left_rear_leg_2].angle), min(servo_max,max(servo_min, 180 - left_rear_leg_2_adjust - walk_leg_2_cycle_1)), -walk_speed_increment)
         if servo.servo[left_rear_leg_3].angle < (180 - left_rear_leg_3_adjust - walk_leg_3_cycle_1 - roll_steady_value + pitch_steady_value):
             lrleg3 = range(math.ceil(servo.servo[left_rear_leg_3].angle), min(servo_max,max(servo_min, 180 - left_rear_leg_3_adjust - walk_leg_3_cycle_1 - roll_steady_value + pitch_steady_value)), walk_speed_increment)
         else:
             lrleg3 = range(math.ceil(servo.servo[left_rear_leg_3].angle), min(servo_max,max(servo_min, 180 - left_rear_leg_3_adjust - walk_leg_3_cycle_1 - roll_steady_value + pitch_steady_value)), -walk_speed_increment)
         if debug == 'on':    
             print('rfleg1', rfleg1, 'rfleg2', rfleg2, 'rfleg3', rfleg3, 'lfleg1', lfleg1, 'lfleg2', lfleg2, 'lfleg3', lfleg3, 'rrleg1', rrleg1, 'rrleg2', rrleg2, 'rrleg3', rrleg3, 'lrleg1', lrleg1, 'lrleg2', lrleg2, 'lrleg3', lrleg3)
         for (a, b, c, d, e, f, g, h, i, j, k, l) in itertools.zip_longest(rfleg1, rfleg2, rfleg3, lfleg1, lfleg2, lfleg3, rrleg1, rrleg2, rrleg3, lrleg1, lrleg2, lrleg3,fillvalue='finished'):
             if a == 'finished':
                 servo.servo[right_front_leg_1].angle = min(servo_max,max(servo_min, right_front_leg_1_adjust + standing_leg_1 - roll_steady_value))
             else:
                 servo.servo[right_front_leg_1].angle = a
             if b == 'finished':
                 servo.servo[right_front_leg_2].angle = min(servo_max,max(servo_min, right_front_leg_2_adjust + walk_leg_2_cycle_1 - lean_adjust_2)) 
             else:
                 servo.servo[right_front_leg_2].angle = b
             if c == 'finished':
                 servo.servo[right_front_leg_3].angle = min(servo_max,max(servo_min, right_front_leg_3_adjust + walk_leg_3_cycle_1 + roll_steady_value + pitch_steady_value + lean_adjust_3))
             else:
                 servo.servo[right_front_leg_3].angle = c
             if d == 'finished':
                 servo.servo[left_front_leg_1].angle = min(servo_max,max(servo_min, 180 - left_front_leg_1_adjust - standing_leg_1 - roll_steady_value))
             else:
                 servo.servo[left_front_leg_1].angle = d
             if e == 'finished':
                 servo.servo[left_front_leg_2].angle = min(servo_max,max(servo_min, 180 - left_front_leg_2_adjust - walk_leg_2_cycle_2 + lean_adjust_2))
             else:
                 servo.servo[left_front_leg_2].angle = e
             if f == 'finished':
                 servo.servo[left_front_leg_3].angle = min(servo_max,max(servo_min, 180 - left_front_leg_3_adjust - walk_leg_3_cycle_2 - roll_steady_value - pitch_steady_value - lean_adjust_3))
             else:
                 servo.servo[left_front_leg_3].angle = f
             if g == 'finished':
                 servo.servo[right_rear_leg_1].angle = min(servo_max,max(servo_min, 180 - right_rear_leg_1_adjust - standing_leg_1 + roll_steady_value))
             else:
                 servo.servo[right_rear_leg_1].angle = g
             if h == 'finished':
                 servo.servo[right_rear_leg_2].angle = min(servo_max,max(servo_min, right_rear_leg_2_adjust + walk_leg_2_cycle_2))
             else:
                 servo.servo[right_rear_leg_2].angle = h
             if i == 'finished':
                 servo.servo[right_rear_leg_3].angle = min(servo_max,max(servo_min, right_rear_leg_3_adjust + walk_leg_3_cycle_2 + roll_steady_value - pitch_steady_value))
             else:
                 servo.servo[right_rear_leg_3].angle = i
             if j == 'finished':
                 servo.servo[left_rear_leg_1].angle = min(servo_max,max(servo_min, left_rear_leg_1_adjust + standing_leg_1 + roll_steady_value))
             else:
                 servo.servo[left_rear_leg_1].angle = j
             if k == 'finished':
                 servo.servo[left_rear_leg_2].angle = min(servo_max,max(servo_min, 180 - left_rear_leg_2_adjust - walk_leg_2_cycle_1))
             else:
                 servo.servo[left_rear_leg_2].angle = k
             if l == 'finished':
                 servo.servo[left_rear_leg_3].angle = min(servo_max,max(servo_min, 180 - left_rear_leg_3_adjust - walk_leg_3_cycle_1 - roll_steady_value + pitch_steady_value))
             else:
                 servo.servo[left_rear_leg_3].angle = l
             if debug == 'on':
                 print(
              #         servo.servo[right_front_leg_1].angle,
              #         servo.servo[right_front_leg_2].angle,
              #         servo.servo[right_front_leg_3].angle,
              #         servo.servo[left_front_leg_1].angle,
              #         servo.servo[left_front_leg_2].angle,
              #         servo.servo[left_front_leg_3].angle,
                       servo.servo[right_rear_leg_1].angle,
                       servo.servo[right_rear_leg_2].angle,
                       servo.servo[right_rear_leg_3].angle,
              #         servo.servo[left_rear_leg_1].angle,
              #         servo.servo[left_rear_leg_2].angle,
              #        servo.servo[left_rear_leg_3].angle,
                       'cycle count', cycle_count)
             #time.sleep(walk_speed_delay)
         cycle_count = cycle_count+1
         print('cycle count', cycle_count)
         if cycle_count > 9:
             cycle_count = 0
             step_count = step_count+2
         
         dist_right = math.ceil(ultrasonicright.distance*100)
         dist_left = math.ceil(ultrasonicleft.distance*100)
         if debug == 'on':
             print('distance_r', dist_right,'distance_r', dist_left)
         if dist_right in range (1, 15):
            if dist_left in range (1, 15):
                print('WALL! Stop!', ultrasonicright.distance, ultrasonicleft.distance)
                break
         input_state = GPIO.input(SW1)
         if input_state == False:
            print ('Single press stopping')
            break  
         else:
            GPIO.output(LED1,GPIO.LOW)
         BackGroundTasks()
        
def GetUp1():
        time.sleep(2)
        GPIO.output(LED1,GPIO.HIGH)
        servo.servo[left_front_leg_2].angle = left_front_leg_2_adjust + servo_min
        time.sleep(1)
        servo.servo[left_front_leg_2].angle = left_front_leg_2_adjust + servo_max
        GPIO.output(LED1,GPIO.LOW)
        time.sleep(2)
        StandingUp

def GetUp2():
        time.sleep(2)
        GPIO.output(LED1,GPIO.HIGH)
        servo.servo[right_front_leg_2].angle = right_front_leg_2_adjust + servo_min
        time.sleep(1)
        servo.servo[right_front_leg_2].angle = right_front_leg_2_adjust + servo_max
        GPIO.output(LED1,GPIO.LOW)
        time.sleep(2)
        StandingUp()
# Set Standing Normally

def StartLieDown():
       servo.servo[left_front_leg_1].angle = 180 - left_front_leg_1_adjust - lie_leg_1
       servo.servo[left_front_leg_2].angle = 180 - left_front_leg_2_adjust - lie_leg_2
       servo.servo[left_front_leg_3].angle = 180 - left_front_leg_3_adjust - lie_leg_3
       
       servo.servo[right_front_leg_1].angle = right_front_leg_1_adjust + lie_leg_1
       servo.servo[right_front_leg_2].angle = right_front_leg_2_adjust + lie_leg_2
       servo.servo[right_front_leg_3].angle = right_front_leg_3_adjust + lie_leg_3
       
       servo.servo[left_rear_leg_1].angle = left_rear_leg_1_adjust + lie_leg_1
       servo.servo[left_rear_leg_2].angle = 180 - left_rear_leg_2_adjust - lie_leg_2
       servo.servo[left_rear_leg_3].angle = 180 - left_rear_leg_3_adjust - lie_leg_3
       
       servo.servo[right_rear_leg_1].angle = 180 - right_rear_leg_1_adjust - lie_leg_1
       servo.servo[right_rear_leg_2].angle = right_rear_leg_2_adjust + lie_leg_2
       servo.servo[right_rear_leg_3].angle = right_rear_leg_3_adjust + lie_leg_3
       
def LieDown():
    while True:
    
        if servo.servo[right_front_leg_1].angle < right_front_leg_1_adjust + lie_leg_1:
             rfleg1 = range(math.ceil(servo.servo[right_front_leg_1].angle), right_front_leg_1_adjust + lie_leg_1, lie_speed_increment)
        else:
             rfleg1 = range(math.ceil(servo.servo[right_front_leg_1].angle), right_front_leg_1_adjust + lie_leg_1, -lie_speed_increment)
             
        if servo.servo[right_front_leg_2].angle < right_front_leg_2_adjust + lie_leg_2:
             rfleg2 = range(math.ceil(servo.servo[right_front_leg_2].angle), right_front_leg_2_adjust + lie_leg_2, lie_speed_increment)
        else:
             rfleg2 = range(math.ceil(servo.servo[right_front_leg_2].angle), right_front_leg_2_adjust + lie_leg_2, -lie_speed_increment)
             
        if servo.servo[right_front_leg_3].angle < right_front_leg_3_adjust + lie_leg_3:
             rfleg3 = range(math.ceil(servo.servo[right_front_leg_3].angle), right_front_leg_3_adjust + lie_leg_3, lie_speed_increment)
        else:
             rfleg3 = range(math.ceil(servo.servo[right_front_leg_3].angle), right_front_leg_3_adjust + lie_leg_3, -lie_speed_increment) 
        
        if servo.servo[right_rear_leg_1].angle < 180 - right_rear_leg_1_adjust - lie_leg_1:
             rrleg1 = range(math.ceil(servo.servo[right_rear_leg_1].angle), 180 - right_rear_leg_1_adjust - lie_leg_1, lie_speed_increment)
        else:
             rrleg1 = range(math.ceil(servo.servo[right_rear_leg_1].angle), 180 - right_rear_leg_1_adjust - lie_leg_1, -lie_speed_increment)
             
        if servo.servo[right_rear_leg_2].angle < right_rear_leg_2_adjust + lie_leg_2:
             rrleg2 = range(math.ceil(servo.servo[right_rear_leg_2].angle), right_rear_leg_2_adjust + lie_leg_2, lie_speed_increment)
        else:
             rrleg2 = range(math.ceil(servo.servo[right_rear_leg_2].angle), right_rear_leg_2_adjust + lie_leg_2, -lie_speed_increment)
             
        if servo.servo[right_rear_leg_3].angle < right_rear_leg_3_adjust + lie_leg_2:
             rrleg3 = range(math.ceil(servo.servo[right_rear_leg_3].angle), right_rear_leg_3_adjust + lie_leg_3, lie_speed_increment)
        else:
             rrleg3 = range(math.ceil(servo.servo[right_rear_leg_3].angle), right_rear_leg_3_adjust + lie_leg_3, -lie_speed_increment)
        
        if servo.servo[left_rear_leg_1].angle < left_rear_leg_1_adjust + lie_leg_1:
             lrleg1 = range(math.ceil(servo.servo[left_rear_leg_1].angle), left_rear_leg_1_adjust + lie_leg_1, lie_speed_increment)
        else:
             lrleg1 = range(math.ceil(servo.servo[left_rear_leg_1].angle), left_rear_leg_1_adjust + lie_leg_1, -lie_speed_increment)
             
        if servo.servo[left_rear_leg_2].angle < 180 - left_rear_leg_2_adjust - lie_leg_2:
             lrleg2 = range(math.ceil(servo.servo[left_rear_leg_2].angle), 180 - left_rear_leg_2_adjust - lie_leg_2, lie_speed_increment)
        else:
             lrleg2 = range(math.ceil(servo.servo[left_rear_leg_2].angle), 180 - left_rear_leg_2_adjust - lie_leg_2, -lie_speed_increment)
             
        if servo.servo[left_rear_leg_3].angle < 180 - left_rear_leg_3_adjust - lie_leg_3:
             lrleg3 = range(math.ceil(servo.servo[left_rear_leg_3].angle), 180 - left_rear_leg_3_adjust - lie_leg_3, lie_speed_increment)
        else:
             lrleg3 = range(math.ceil(servo.servo[left_rear_leg_3].angle), 180 - left_rear_leg_3_adjust - lie_leg_3, -lie_speed_increment) 
        
        if servo.servo[left_front_leg_1].angle < 180 - left_front_leg_1_adjust - lie_leg_1:
             lfleg1 = range(math.ceil(servo.servo[left_front_leg_1].angle), 180 - left_front_leg_1_adjust - lie_leg_1, lie_speed_increment)
        else:
             lfleg1 = range(math.ceil(servo.servo[left_front_leg_1].angle), 180 - left_front_leg_1_adjust - lie_leg_1, -lie_speed_increment)
             
        if servo.servo[left_front_leg_2].angle < 180 - left_front_leg_2_adjust - lie_leg_2:
             lfleg2 = range(math.ceil(servo.servo[left_front_leg_2].angle), 180 - left_front_leg_2_adjust - lie_leg_2, lie_speed_increment)
        else:
             lfleg2 = range(math.ceil(servo.servo[left_front_leg_2].angle), 180 - left_front_leg_2_adjust - lie_leg_2, -lie_speed_increment)
             
        if servo.servo[left_front_leg_3].angle < 180 - left_front_leg_3_adjust - lie_leg_3:
             lfleg3 = range(math.ceil(servo.servo[left_front_leg_3].angle), 180 - left_front_leg_3_adjust - lie_leg_3, lie_speed_increment)
        else:
             lfleg3 = range(math.ceil(servo.servo[left_front_leg_3].angle), 180 - left_front_leg_3_adjust - lie_leg_3, -lie_speed_increment) 
        
        for (a, b, c, d, e, f, g, h, i, j, k, l) in itertools.zip_longest(rfleg1, rfleg2, rfleg3, lfleg1, lfleg2, lfleg3, rrleg1, rrleg2, rrleg3, lrleg1, lrleg2, lrleg3,fillvalue='finished'):
             if a == 'finished':
                 servo.servo[right_front_leg_1].angle = right_front_leg_1_adjust + lie_leg_1
             else:
                 servo.servo[right_front_leg_1].angle = a
             if b == 'finished':
                 servo.servo[right_front_leg_2].angle = right_front_leg_2_adjust + lie_leg_2
             else:
                 servo.servo[right_front_leg_2].angle = b
             if c == 'finished':
                 servo.servo[right_front_leg_3].angle = right_front_leg_3_adjust + lie_leg_3
             else:
                 servo.servo[right_front_leg_3].angle = c
             if d == 'finished':
                 servo.servo[left_front_leg_1].angle = 180 - left_front_leg_1_adjust - lie_leg_1
             else:
                 servo.servo[left_front_leg_1].angle = d
             if e == 'finished':
                 servo.servo[left_front_leg_2].angle = 180 - left_front_leg_2_adjust - lie_leg_2
             else:
                 servo.servo[left_front_leg_2].angle = e
             if f == 'finished':
                 servo.servo[left_front_leg_3].angle = 180 - left_front_leg_3_adjust - lie_leg_3
             else:
                 servo.servo[left_front_leg_3].angle = f
             if g == 'finished':
                 servo.servo[right_rear_leg_1].angle = 180 - right_rear_leg_1_adjust - lie_leg_1
             else:
                 servo.servo[right_rear_leg_1].angle = g
             if h == 'finished':
                 servo.servo[right_rear_leg_2].angle = right_rear_leg_2_adjust + lie_leg_2
             else:
                 servo.servo[right_rear_leg_2].angle = h
             if i == 'finished':
                 servo.servo[right_rear_leg_3].angle = right_rear_leg_3_adjust + lie_leg_3
             else:
                 servo.servo[right_rear_leg_3].angle = i
             if j == 'finished':
                 servo.servo[left_rear_leg_1].angle = left_rear_leg_1_adjust + lie_leg_1
             else:
                 servo.servo[left_rear_leg_1].angle = j
             if k == 'finished':
                 servo.servo[left_rear_leg_2].angle = 180 - left_rear_leg_2_adjust - lie_leg_2
             else:
                 servo.servo[left_rear_leg_2].angle = k
             if l == 'finished':
                 servo.servo[left_rear_leg_3].angle = 180 - left_rear_leg_3_adjust - lie_leg_3
             else:
                 servo.servo[left_rear_leg_3].angle = l
        BackGroundTasks()       
    

def LegTest():
       servo.servo[left_front_leg_1].angle = 180 - left_front_leg_1_adjust - standing_leg_1
       servo.servo[left_front_leg_2].angle = 180 - left_front_leg_2_adjust - standing_leg_2
       servo.servo[left_front_leg_3].angle = 180 - left_front_leg_3_adjust - standing_leg_3
       
       servo.servo[right_front_leg_1].angle = right_front_leg_1_adjust + standing_leg_1
       servo.servo[right_front_leg_2].angle = right_front_leg_2_adjust + standing_leg_2
       servo.servo[right_front_leg_3].angle = right_front_leg_3_adjust + standing_leg_3
       
       servo.servo[left_rear_leg_1].angle = left_rear_leg_1_adjust + standing_leg_1
       servo.servo[left_rear_leg_2].angle = 180 - left_rear_leg_2_adjust - 100
       servo.servo[left_rear_leg_3].angle = 180 - left_rear_leg_3_adjust - 120
       
       servo.servo[right_rear_leg_1].angle = 180 - right_rear_leg_1_adjust - standing_leg_1
       servo.servo[right_rear_leg_2].angle = right_rear_leg_2_adjust + standing_leg_2
       servo.servo[right_rear_leg_3].angle = right_rear_leg_3_adjust + standing_leg_3
       


def StandUp():
       servo.servo[left_front_leg_1].angle = 180 - left_front_leg_1_adjust - standing_leg_1
       servo.servo[left_front_leg_2].angle = 180 - left_front_leg_2_adjust - standing_leg_2
       servo.servo[left_front_leg_3].angle = 180 - left_front_leg_3_adjust - standing_leg_3
       
       servo.servo[right_front_leg_1].angle = right_front_leg_1_adjust + standing_leg_1
       servo.servo[right_front_leg_2].angle = right_front_leg_2_adjust + standing_leg_2
       servo.servo[right_front_leg_3].angle = right_front_leg_3_adjust + standing_leg_3
       
       servo.servo[left_rear_leg_1].angle = left_rear_leg_1_adjust + standing_leg_1
       servo.servo[left_rear_leg_2].angle = 180 - left_rear_leg_2_adjust - standing_leg_2
       servo.servo[left_rear_leg_3].angle = 180 - left_rear_leg_3_adjust - standing_leg_3
       
       servo.servo[right_rear_leg_1].angle = 180 - right_rear_leg_1_adjust - standing_leg_1
       servo.servo[right_rear_leg_2].angle = right_rear_leg_2_adjust + standing_leg_2
       servo.servo[right_rear_leg_3].angle = right_rear_leg_3_adjust + standing_leg_3
       
       StandingUp()
def StandingUp():
        while True:

          heading, roll, pitch = orientation.read_euler()
          #if roll*accuracy in range(accuracy*50,accuracy*500):
           #    print ('oops I fell over+', roll)
            #   GetUp1()
           #break
          #if roll*accuracy in range(accuracy*-500,accuracy*-50):
             #  print ('oops I fell over-', roll)
              # GetUp2()
              #break
          roll_steady_value = (math.ceil(roll_reaction*(roll+roll_adjust)))
          pitch_steady_value = (math.ceil(pitch_reaction*(pitch+pitch_adjust)))
          servo.servo[left_front_leg_1].angle = min(servo_max,max(servo_min, 180 - left_front_leg_1_adjust - standing_leg_1 - roll_steady_value))
          servo.servo[left_front_leg_2].angle = min(servo_max,max(servo_min, 180 - left_front_leg_2_adjust - standing_leg_2 + roll_steady_value + pitch_steady_value))
          servo.servo[left_front_leg_3].angle = min(servo_max,max(servo_min, 180 - left_front_leg_3_adjust - standing_leg_3 - roll_steady_value - pitch_steady_value))
          
          servo.servo[right_front_leg_1].angle = min(servo_max,max(servo_min, right_front_leg_1_adjust + standing_leg_1 - roll_steady_value))
          servo.servo[right_front_leg_2].angle = min(servo_max,max(servo_min, right_front_leg_2_adjust + standing_leg_2 + roll_steady_value - pitch_steady_value))
          servo.servo[right_front_leg_3].angle = min(servo_max,max(servo_min, right_front_leg_3_adjust + standing_leg_3 - roll_steady_value + pitch_steady_value))
       
          servo.servo[left_rear_leg_1].angle = min(servo_max,max(servo_min, left_rear_leg_1_adjust + standing_leg_1 + roll_steady_value))
          servo.servo[left_rear_leg_2].angle = min(servo_max,max(servo_min, 180 - left_rear_leg_2_adjust - standing_leg_2 + roll_steady_value - pitch_steady_value))
          servo.servo[left_rear_leg_3].angle = min(servo_max,max(servo_min, 180 - left_rear_leg_3_adjust - standing_leg_3 - roll_steady_value + pitch_steady_value))
       
          servo.servo[right_rear_leg_1].angle = min(servo_max,max(servo_min, 180 - right_rear_leg_1_adjust - standing_leg_1 + roll_steady_value))
          servo.servo[right_rear_leg_2].angle = min(servo_max,max(servo_min, right_rear_leg_2_adjust + standing_leg_2 + roll_steady_value + pitch_steady_value))
          servo.servo[right_rear_leg_3].angle = min(servo_max,max(servo_min, right_rear_leg_3_adjust + standing_leg_3 - roll_steady_value - pitch_steady_value))
          
          if debug == 'on':
           print('roll', roll)
           print('sensor data', orientation.read_euler())
          BackGroundTasks()                  
       
def Sit():
    while True:
        
        if servo.servo[right_front_leg_1].angle < right_front_leg_1_adjust + sit_fleg_1:
             rfleg1 = range(math.ceil(servo.servo[right_front_leg_1].angle), right_front_leg_1_adjust + sit_fleg_1, sit_speed_increment)
        else:
             rfleg1 = range(math.ceil(servo.servo[right_front_leg_1].angle), right_front_leg_1_adjust + sit_fleg_1, -sit_speed_increment)
             
        if servo.servo[right_front_leg_2].angle < right_front_leg_2_adjust + sit_fleg_2:
             rfleg2 = range(math.ceil(servo.servo[right_front_leg_2].angle), right_front_leg_2_adjust - sit_fleg_2, sit_speed_increment)
        else:
             rfleg2 = range(math.ceil(servo.servo[right_front_leg_2].angle), right_front_leg_2_adjust - sit_fleg_2, -sit_speed_increment)
             
        if servo.servo[right_front_leg_3].angle < right_front_leg_3_adjust + sit_fleg_3:
             rfleg3 = range(math.ceil(servo.servo[right_front_leg_3].angle), right_front_leg_3_adjust + sit_fleg_3, sit_speed_increment)
        else:
             rfleg3 = range(math.ceil(servo.servo[right_front_leg_3].angle), right_front_leg_3_adjust + sit_fleg_3, -sit_speed_increment) 
        
        if servo.servo[right_rear_leg_1].angle < 180 - right_rear_leg_1_adjust - sit_rleg_1:
             rrleg1 = range(math.ceil(servo.servo[right_rear_leg_1].angle), 180 - right_rear_leg_1_adjust - sit_rleg_1, sit_speed_increment)
        else:
             rrleg1 = range(math.ceil(servo.servo[right_rear_leg_1].angle), 180 - right_rear_leg_1_adjust - sit_rleg_1, -sit_speed_increment)
             
        if servo.servo[right_rear_leg_2].angle < right_rear_leg_2_adjust + sit_rleg_2:
             rrleg2 = range(math.ceil(servo.servo[right_rear_leg_2].angle), right_rear_leg_2_adjust + sit_rleg_2, sit_speed_increment)
        else:
             rrleg2 = range(math.ceil(servo.servo[right_rear_leg_2].angle), right_rear_leg_2_adjust + sit_rleg_2, -sit_speed_increment)
             
        if servo.servo[right_rear_leg_3].angle < right_rear_leg_3_adjust + sit_rleg_2:
             rrleg3 = range(math.ceil(servo.servo[right_rear_leg_3].angle), right_rear_leg_3_adjust + sit_rleg_3, sit_speed_increment)
        else:
             rrleg3 = range(math.ceil(servo.servo[right_rear_leg_3].angle), right_rear_leg_3_adjust + sit_rleg_3, -sit_speed_increment)
        
        if servo.servo[left_rear_leg_1].angle < left_rear_leg_1_adjust + sit_rleg_1:
             lrleg1 = range(math.ceil(servo.servo[left_rear_leg_1].angle), left_rear_leg_1_adjust + sit_rleg_1, sit_speed_increment)
        else:
             lrleg1 = range(math.ceil(servo.servo[left_rear_leg_1].angle), left_rear_leg_1_adjust + sit_rleg_1, -sit_speed_increment)
             
        if servo.servo[left_rear_leg_2].angle < 180 - left_rear_leg_2_adjust - sit_rleg_2:
             lrleg2 = range(math.ceil(servo.servo[left_rear_leg_2].angle), 180 - left_rear_leg_2_adjust - sit_rleg_2, sit_speed_increment)
        else:
             lrleg2 = range(math.ceil(servo.servo[left_rear_leg_2].angle), 180 - left_rear_leg_2_adjust - sit_rleg_2, -sit_speed_increment)
             
        if servo.servo[left_rear_leg_3].angle < 180 - left_rear_leg_3_adjust - sit_rleg_3:
             lrleg3 = range(math.ceil(servo.servo[left_rear_leg_3].angle), 180 - left_rear_leg_3_adjust - sit_rleg_3, sit_speed_increment)
        else:
             lrleg3 = range(math.ceil(servo.servo[left_rear_leg_3].angle), 180 - left_rear_leg_3_adjust - sit_rleg_3, -sit_speed_increment) 
        
        if servo.servo[left_front_leg_1].angle < 180 - left_front_leg_1_adjust - sit_fleg_1:
             lfleg1 = range(math.ceil(servo.servo[left_front_leg_1].angle), 180 - left_front_leg_1_adjust - sit_fleg_1, sit_speed_increment)
        else:
             lfleg1 = range(math.ceil(servo.servo[left_front_leg_1].angle), 180 - left_front_leg_1_adjust - sit_fleg_1, -sit_speed_increment)
             
        if servo.servo[left_front_leg_2].angle < 180 - left_front_leg_2_adjust - sit_fleg_2:
             lfleg2 = range(math.ceil(servo.servo[left_front_leg_2].angle), 180 - left_front_leg_2_adjust - sit_fleg_2, sit_speed_increment)
        else:
             lfleg2 = range(math.ceil(servo.servo[left_front_leg_2].angle), 180 - left_front_leg_2_adjust - sit_fleg_2, -sit_speed_increment)
             
        if servo.servo[left_front_leg_3].angle < 180 - left_front_leg_3_adjust - sit_fleg_3:
             lfleg3 = range(math.ceil(servo.servo[left_front_leg_3].angle), 180 - left_front_leg_3_adjust - sit_fleg_3, sit_speed_increment)
        else:
             lfleg3 = range(math.ceil(servo.servo[left_front_leg_3].angle), 180 - left_front_leg_3_adjust - sit_fleg_3, -sit_speed_increment) 
        
        for (a, b, c, d, e, f, g, h, i, j, k, l) in itertools.zip_longest(rfleg1, rfleg2, rfleg3, lfleg1, lfleg2, lfleg3, rrleg1, rrleg2, rrleg3, lrleg1, lrleg2, lrleg3,fillvalue='finished'):
             print(a, b, c, d, e, f, g, h, i, j, k, l)
             if a == 'finished':
                 servo.servo[right_front_leg_1].angle = right_front_leg_1_adjust + sit_fleg_1
             else:
                 servo.servo[right_front_leg_1].angle = a
             if b == 'finished':
                 servo.servo[right_front_leg_2].angle = right_front_leg_2_adjust + sit_fleg_2
             else:
                 servo.servo[right_front_leg_2].angle = b
             if c == 'finished':
                 servo.servo[right_front_leg_3].angle = right_front_leg_3_adjust + sit_fleg_2
             else:
                 servo.servo[right_front_leg_3].angle = c
             if d == 'finished':
                 servo.servo[left_front_leg_1].angle = 180 - left_front_leg_1_adjust - sit_fleg_1
             else:
                 servo.servo[left_front_leg_1].angle = d
             if e == 'finished':
                 servo.servo[left_front_leg_2].angle = 180 - left_front_leg_2_adjust - sit_fleg_2
             else:
                 servo.servo[left_front_leg_2].angle = e
             if f == 'finished':
                 servo.servo[left_front_leg_3].angle = 180 - left_front_leg_3_adjust - sit_fleg_3
             else:
                 servo.servo[left_front_leg_3].angle = f
             if g == 'finished':
                 servo.servo[right_rear_leg_1].angle = 180 - right_rear_leg_1_adjust - sit_rleg_1
             else:
                 servo.servo[right_rear_leg_1].angle = g
             if h == 'finished':
                 servo.servo[right_rear_leg_2].angle = right_rear_leg_2_adjust + sit_rleg_2
             else:
                 servo.servo[right_rear_leg_2].angle = h
             if i == 'finished':
                 servo.servo[right_rear_leg_3].angle = right_rear_leg_3_adjust + sit_rleg_3
             else:
                 servo.servo[right_rear_leg_3].angle = i
             if j == 'finished':
                 servo.servo[left_rear_leg_1].angle = left_rear_leg_1_adjust + sit_rleg_1
             else:
                 servo.servo[left_rear_leg_1].angle = j
             if k == 'finished':
                 servo.servo[left_rear_leg_2].angle = 180 - left_rear_leg_2_adjust - sit_rleg_2
             else:
                 servo.servo[left_rear_leg_2].angle = k
             if l == 'finished':
                 servo.servo[left_rear_leg_3].angle = 180 - left_rear_leg_3_adjust - sit_rleg_3
             else:
                 servo.servo[left_rear_leg_3].angle = l
        BackGroundTasks()           
           

def Buttons():
#  while True:
    if GPIO.event_detected(SW1):
             GPIO.remove_event_detect(SW1)
             now = time.time()
             count = 1
             GPIO.add_event_detect(SW1,GPIO.RISING)
             while time.time() < now + 1: # 1 second period
                if GPIO.event_detected(SW1):
                   count +=1
                   time.sleep(.40) # debounce time
#              print count
#              performing required task!
#             if count == 1:
#                 print ('Long press man resetting orientation')
#                 GPIO.output(orientation_rst,GPIO.HIGH)
#                 flashLED(0.05, 100)
#                 GPIO.output(orientation_rst,GPIO.LOW)
             if count == 2:
              singlePress()
              GPIO.remove_event_detect(SW1)
              GPIO.add_event_detect(SW1,GPIO.FALLING)
             elif count == 3:
              doublePress()
              GPIO.remove_event_detect(SW1)
              GPIO.add_event_detect(SW1,GPIO.FALLING)
             elif count == 4:
              triplePress()
              GPIO.remove_event_detect(SW1)
              GPIO.add_event_detect(SW1,GPIO.FALLING)
             elif count == 5:
              QuadPress()
              GPIO.remove_event_detect(SW1)
              GPIO.add_event_detect(SW1,GPIO.FALLING)
        

def VoiceCommand():
# obtain audio from the microphone
  try:

   r = sr.Recognizer()
   with sr.Microphone(chunk_size=1024, sample_rate=48000) as source:
  #with sr.Microphone as source:
    print("Say something!")
    GPIO.output(LED1,GPIO.HIGH)
    audio = r.listen(source,timeout=4)
    print("Done")
    GPIO.output(LED1,GPIO.LOW)
    
      
    # recognize speech using Google Cloud Speech
   with open("/home/pi/SpotPupPi/SpotPuppy-GoogleSpeach.json") as f:
    GOOGLE_CLOUD_SPEECH_CREDENTIALS = f.read()
    
    print("Google Cloud Speech thinks you said " + r.recognize_google_cloud(audio, credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS))
    if 'down' in r.recognize_google_cloud(audio, credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS):
      Sit()
      GPIO.output(LED1,GPIO.LOW)
    if 'walk forward' in r.recognize_google_cloud(audio, credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS):
      Walk()
    if 'lie down' in r.recognize_google_cloud(audio, credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS):
      Liedown()
    if 'stand up' in r.recognize_google_cloud(audio, credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS):
      StandUp()
  except sr.WaitTimeoutError:
      print("Timeout")
      BackGroundTasks()
  except sr.UnknownValueError:
      print("Google Cloud Speech could not understand audio")
      BackGroundTasks()
  except sr.RequestError as e:
      print("Could not request results from Google Cloud Speech service; {0}".format(e))
      BackGroundTasks()

def BackGroundTasks():
    dist = math.ceil(ultrasonicleft.distance*100)
    GPIO.output(LED1,GPIO.LOW)
    print(dist)
    try:
        Buttons()
        if dist in range (1, 15):
            print('Hay watch out!', ultrasonicleft.distance*100)
            if dist < 10:
                VoiceCommand()
        temp_c = orientation.read_temp()
          
        if temp_c >48:
           print('Too Hot!!- Temp:', temp_c)
           GPIO.output(RELAY1,GPIO.HIGH)
           GPIO.output(RELAY2,GPIO.HIGH)
           
    except KeyboardInterrupt:
            print("Puppy stopped by User")
            GPIO.cleanup()

#Run Once
#if not orientation.begin(): 
print('Start Point')
GPIO.output(RELAY1,GPIO.HIGH)
GPIO.output(RELAY2,GPIO.HIGH)
# Initialize orientation and stop if something went wrong.
if not orientation.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = orientation.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = orientation.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

# Start PupPi lying down
StartLieDown()


#Run
if __name__ == '__main__':
        while True:
          BackGroundTasks()
          #Buttons()
          #StandUp()
          #LegTest()
          #StandingUp()
          #
          #Walk2()
          #LieDown()
          
          #print(distance)
          

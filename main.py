# UPYBBOT - micropython balaancing robot

import esp32
import machine 
import time
from time import ticks_us,ticks_cpu
from machine import Pin
from machine import Timer
# set up stepper motors

from nemastepper import Stepper
from nemastepper import Stepper
motor1 = Stepper(26,25,12)
motor2 = Stepper(33,32,14)

def step_cb(t):
    motor1.do_step()
    motor2.do_step()
    
timer=Timer(8)
timer.init(freq=10000, mode=Timer.PERIODIC, callback=step_cb)   #initializing the timer

motor1.MAX_ACCEL = 1000  
motor2.MAX_ACCEL = 1000  
speed = 3000
motor1.set_speed(speed)
motor2.set_speed(speed)
print ('press button to stop')
while 1 :
    print (speed)
print ('button pressed')
    
motor1.set_speed(0)
motor1.set_off()

timer.deinit()


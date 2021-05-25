# UPYBBOT - micropython balaancing robot
# Complete project details at https://RandomNerdTutorials.com
# esptool.py erase_flash
# esptool.py --port COM3 write_flash 0x1000 esp32-idf3-20210202-v1.14.bin

from time import sleep
import machine 
from machine import Pin
from machine import Timer

led = machine.Pin(19, machine.Pin.OUT)

# set up stepper motors
from nemastepper import Stepper
motor1 = Stepper(26,25,12)
motor2 = Stepper(33,32,14)

def step_cb():
    motor1.do_step()
    motor2.do_step()

timer=Timer(4)
timer.init(freq=10000, mode=Timer.PERIODIC, callback=step_cb)   #initializing the timer

motor1.MAX_ACCEL = 1000  
motor2.MAX_ACCEL = 1000  
speed = 3000
motor1.set_speed(speed)
motor2.set_speed(speed)
print ('press button to stop')

while True :
  
  led.value(1)            #Set led turn on
  sleep(0.1)
  led.value(0)            #Set led turn off
  sleep(0.1)
  #step_cb()
  
print ('end')
    
motor1.set_speed(0)
motor1.set_off()
motor2.set_speed(0)
motor2.set_off()
timer.deinit()



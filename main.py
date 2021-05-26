# UPYBBOT - micropython balancing robot
# cd C:\Python39\Lib\site-packages>
# C:\Python39\Lib\site-packages>esptool.py erase_flash
# C:\Python39\Lib\site-packages>esptool.py --port COM3 write_flash 0x1000 C:\ESP32a\esp32-idf3-20210202-v1.14.bin
import machine 
import time
from time import ticks_us,ticks_cpu
from machine import Pin
from machine import Timer
# set up stepper motors

from nemastepper import Stepper

motor1 = Stepper(26,25,12)
motor2 = Stepper(33,32,14)
led = Pin(19, Pin.OUT)

def step_cb(self):
    motor1.do_step()
    motor2.do_step()

def led_cb(self):
    led.value(not led.value())

timer=Timer(5)
timer.init(freq=1, mode=Timer.PERIODIC, callback=led_cb)   #initializing the timer

timer2=Timer(8)
timer2.init(freq=10000, mode=Timer.PERIODIC, callback=step_cb)   #initializing the timer


motor1.MAX_ACCEL = 1000  
motor2.MAX_ACCEL = 1000  
speed = 3000
motor1.set_speed(speed)
motor2.set_speed(speed)
print ('press button to stop')
while 1 :
    time.sleep_us(1)    

print ('button pressed')
    
motor1.set_speed(0)
motor1.set_off()
motor2.set_speed(0)
motor2.set_off()
timer.deinit()
timer2.deinit()
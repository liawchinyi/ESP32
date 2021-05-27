# UPYBBOT - micropython balancing robot
# cd C:\Python39\Lib\site-packages
# C:\Python39\Lib\site-packages>esptool.py erase_flash
# C:\Python39\Lib\site-packages>esptool.py --port COM3 write_flash 0x1000 C:\ESP32a\esp32-idf3-20210202-v1.14.bin

# Complete project details at https://RandomNerdTutorials.com

import wifimgr
import time
from time import ticks_us,ticks_cpu, sleep
import machine
from machine import I2C, Pin, Timer, sleep

import mpu6050
i2c = I2C(scl=Pin(22), sda=Pin(21))     #initializing the I2C method for ESP32
mpu= mpu6050.accel(i2c)

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
timer.init(freq=3, mode=Timer.PERIODIC, callback=led_cb)   #initializing the timer

timer2=Timer(8)
timer2.init(freq=10000, mode=Timer.PERIODIC, callback=step_cb)   #initializing the timer

motor1.MAX_ACCEL = 1000  
motor2.MAX_ACCEL = 1000  
speed = 3000
motor1.set_speed(speed)
motor2.set_speed(speed)

#timer3=Timer(7)
#timer3.init(freq=10, mode=Timer.PERIODIC, callback=update_socket)   #initializing the timer

print ('start')

while True:
    mpu.get_values()
    print(mpu.get_values())
    sleep(500)

print ('exit')
    
motor1.set_speed(0)
motor1.set_off()
motor2.set_speed(0)
motor2.set_off()
timer.deinit()
timer2.deinit()
#timer3.deinit()
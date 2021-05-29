# UPYBBOT - micropython balancing robot
# cd C:\Python39\Lib\site-packages
# https://microcontrollerslab.com/micropython-mpu-6050-esp32-esp8266/
# C:\Python39\Lib\site-packages>esptool.py erase_flash
# C:\Python39\Lib\site-packages>esptool.py --port COM3 write_flash 0x1000 C:\ESP32a\esp32-20210418-v1.15.bin
# Complete project details at https://RandomNerdTutorials.com

import time
from time import ticks_us,ticks_cpu, sleep
import machine
from machine import I2C, Pin, Timer, sleep

import mpu6050
i2c = I2C(scl = Pin(22), sda = Pin(21), freq = 400000)
#initializing the I2C method for ESP32
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

#initializing the timer
timer=Timer(3)
timer.init(freq=1, mode=Timer.PERIODIC, callback=led_cb)   #initializing the timer

timer2=Timer(6)
timer2.init(freq=10000, mode=Timer.PERIODIC, callback=step_cb)   #initializing the timer

motor1.MAX_ACCEL = 1000  
motor2.MAX_ACCEL = 1000  
speed = 3000
motor1.set_speed(speed)
motor2.set_speed(speed)

print ('start')
count = 0
while count < 1000:
    count = count + 1
    array = mpu.get_values()
    speed = array["AcX"]
    motor1.set_speed(speed)
    motor2.set_speed(speed)
    print(count,speed, array)
    sleep(500)

print ('exit')
    
motor1.set_speed(0)
motor1.set_off()
motor2.set_speed(0)
motor2.set_off()
timer.deinit()
timer2.deinit()


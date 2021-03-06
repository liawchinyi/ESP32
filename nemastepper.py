import machine
from machine import Pin, PWM
import time
from time import ticks_us, ticks_cpu, sleep, sleep_us, sleep_ms

class Stepper():
    """
    Handles  A4988 hardware driver for bipolar stepper motors
    """
    def __init__(self, dir_pin, step_pin, enable_pin):
        self.step_pin = Pin(step_pin, Pin.OUT)
        self.pwm_pin = PWM(self.step_pin)
        self.dir_pin = Pin(dir_pin, Pin.OUT)
        self.enable_pin = Pin(enable_pin, Pin.OUT)
        self.enable_pin.on()      
        self.dir = 0
        self.pulserate = 100
        self.count = 0
        self.speed = 0
        self.MAX_ACCEL = 30   #equivallent to 100 x (periodicity of set_speed) usteps/sec/sec
 
    def do_step(self):   # called by timer interrupt every 100us
        if self.dir == 0:
            return
        self.count = (self.count+1)%(self.pulserate)        
        
    def set_speed(self, speed): #called periodically
        if (self.speed - speed) > self.MAX_ACCEL:
            self.speed -= self.MAX_ACCEL
        elif (self.speed - speed)< -self.MAX_ACCEL:
            self.speed+=self.MAX_ACCEL
        else:
            self.speed = speed
            
        # set direction
        if self.speed>0:
            self.dir = 1
            self.dir_pin.on()
            self.enable_pin.off()       
        elif self.speed<0:
            self.dir = -1
            self.dir_pin.off()
            self.enable_pin.off()       
        else:
            self.dir = 0
        if abs(self.speed)>0:
            self.pulserate = 10000//(abs(self.speed))
            self.pwm_pin.freq(abs(self.speed))

    def set_off(self):
        self.enable_pin.on()

    def get_speed(self):
        return self.speed
                
            
        
            
    




# UPYBBOT - micropython balancing robot
# cd C:\Python39\Lib\site-packages
# https://microcontrollerslab.com/micropython-mpu-6050-esp32-esp8266/
# C:\Python39\Lib\site-packages>esptool.py erase_flash
# C:\Python39\Lib\site-packages>esptool.py --port COM3 write_flash 0x1000 C:\ESP32a\esp32-20210418-v1.15.bin
# Complete project details at https://RandomNerdTutorials.com
# 
import ubluetooth
import time
import sys
from time import ticks_us, ticks_cpu, sleep, sleep_us, sleep_ms
import machine
from machine import I2C, Pin, Timer, sleep, PWM

# set up stepper motors
from nemastepper import Stepper
motor1 = Stepper(26,25,12)#nemastepper
motor2 = Stepper(33,32,14)#nemastepper

led = PWM(Pin(19), 1)

BOOT_sw = Pin(0, Pin.IN, Pin.PULL_UP) #输入红外探头

print('BOOT Pin = ', BOOT_sw.value())

if BOOT_sw.value() == 0:
    print ('program terminated')
    sys.exit()
    
from mpu6050 import mpu6050
imu = mpu6050()

def step_cb(self):
    global motor1, motor2
    motor1.do_step()
    motor2.do_step()

def led_cb(self):
    global led
    led.value(not led.value())

#initializing the timer
#tim=Timer(7)
#tim.init(freq=10000, mode=Timer.PERIODIC, callback=step_cb)

# Complementary Filter A = rt/(rt + dt) where rt is response time, dt = period
def compf(fangle,accel,gyro,looptime,A):
    fangle = A * (fangle + gyro * looptime/1000000) + (1-A) * accel
    return fangle

#set up wifi radio control
#import wifiradio
#radio = wifiradio.WiFiRadio(1)

MAX_VEL = 2000 # 2000 usteps/sec = 500steps/sec = 2.5rps = 150rpm
MAX_ANGLE = 20  # degrees of tilt for speed control

def constrain(val,minv,maxv):
    if val<minv:
        return minv
    elif val>maxv:
        return maxv
    else:
        return val

#stability PD controiller - input is target angle, output is acceleration
K = 6 # 7
Kp = 4.0
Kd = 0.5

def stability(target,current,rate):
    global K,Kp,Kd
    error = target - current
    output = Kp * error - Kd*rate
    return int(K*output)

#speed P controiller - input is target speed, output is inclination angle
KpS = 0.01
def speedcontrol(target,current):
    global KpS
    error = target - current
    output = KpS * error 
    return constrain(output,-MAX_ANGLE,+MAX_ANGLE)

angle = 0.0
rate = 0.0
gangle = 0.0
controlspeed = 0
fspeed = 0
delta = 0
tangle = 0
motor1speed = 0
motor2speed = 0
#main balance loop runs every 5ms
def balance(self):
    global motor1, motor2, imu, angle, rate, motor2speed
    global gangle, controlspeed, fspeed, delta, tangle, motor1speed
    start = ticks_us()
    angle  = imu.pitch() + 4
    rate   = imu.get_gy() + 4
    gangle = compf(gangle, angle, rate, (ticks_us()-start),0.99) 
    if abs(gangle) < 45 and BOOT_sw.value() == 1:  # give up if inclination angle >=45 degrees
        start = ticks_us()
        # speed control
        motor1speed = motor1.get_speed()
        motor2speed = motor2.get_speed()
        actualspeed = (motor1speed+motor2speed)/4
        fspeed = 0.95 * fspeed + 0.05 * actualspeed
        cmd = [0,2] #radio.poll() # cmd[0] is turn speed, cmd[1] is fwd/rev speed
        tangle = speedcontrol(80*cmd[1],fspeed)
        # stability control
        delta = stability(tangle, gangle, rate)
        controlspeed += delta         
        controlspeed = constrain(controlspeed,-MAX_VEL,MAX_VEL)
        # set motor speed
        motor1.set_speed(-controlspeed-int(300*cmd[0]))
        motor2.set_speed(controlspeed+int(300*cmd[0]))
    else :    
        # stop and turn off motors
        motor1.set_speed(0)
        motor2.set_speed(0)
        motor1.set_off()
        motor2.set_off()

print ('start')
delay_start = ticks_us()
print_start = ticks_us()
while BOOT_sw.value() == 1 :
    
    if (ticks_us()-delay_start) > 1000 :
        balance(1)
        delay_start = ticks_us()

    if (ticks_us()-print_start) > 100000 :
        print('TA',tangle,'GA',gangle,'A',angle,'R',rate,'D',delta,'S1',motor1speed,'S2',motor2speed,'FS',fspeed,)
        print_start = ticks_us()

print ('exit')

motor1.set_speed(0)
motor1.set_off()
motor2.set_speed(0)
motor2.set_off()
#tim.deinit()

# UPYBBOT - micropython balancing robot
# cd C:\Python39\Lib\site-packages
# https://microcontrollerslab.com/micropython-mpu-6050-esp32-esp8266/
# C:\Python39\Lib\site-packages>esptool.py erase_flash
# C:\Python39\Lib\site-packages>esptool.py --port COM3 write_flash 0x1000 C:\ESP32a\esp32-20210418-v1.15.bin
# Complete project details at https://RandomNerdTutorials.com
# 
import time
import sys
from time import ticks_us, ticks_cpu, sleep, sleep_us
import machine
from machine import I2C, Pin, Timer, sleep

# set up stepper motors
from nemastepper import Stepper
motor1 = Stepper(26,25,12)#nemastepper
motor2 = Stepper(33,32,14)#nemastepper

led = Pin(19, Pin.OUT)
led.value(1)
BOOT_sw = Pin(0, Pin.IN, Pin.PULL_UP)#输入红外探头

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
#timer=Timer(4)
#timer.init(freq=2, mode=Timer.PERIODIC, callback=led_cb)

tim=Timer(8)
tim.init(freq=10000, mode=Timer.PERIODIC, callback=step_cb)   

# Complementary Filter A = rt/(rt + dt) where rt is response time, dt = period
def compf(fangle,accel,gyro,looptime,A):
    fangle = A * (fangle + gyro * looptime/1000000) + (1-A) * accel
    return fangle

#set up wifi radio control
#import wifiradio
#radio = wifiradio.WiFiRadio(1)

MAX_VEL = 2000 # 2000 usteps/sec = 500steps/sec = 2.5rps = 150rpm
MAX_ANGLE = 10  # degrees of tilt for speed control

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

#main balance loop runs every 5ms
def balance():
    global motor1, motor2, imu
    gangle = 0.0
    start = ticks_us()
    controlspeed = 0
    fspeed = 0
    while abs(gangle) < 45:  # give up if inclination angle >=45 degrees
        angle  = imu.pitch()
        rate   = imu.get_gy()        
        gangle = compf(gangle, angle, rate, (ticks_us()-start),0.99)         
        start = ticks_us()
        # speed control
        actualspeed = (motor1.get_speed()+motor2.get_speed())/2
        fspeed = 0.95 * fspeed + 0.05 * actualspeed
        cmd = [0,10]#radio.poll() # cmd[0] is turn speed, cmd[1] is fwd/rev speed
        tangle = speedcontrol(800*cmd[1],fspeed)
         # stability control
        controlspeed += stability(tangle, gangle, rate)           
        controlspeed = constrain(controlspeed,-MAX_VEL,MAX_VEL)
        # set motor speed
        motor1.set_speed(-controlspeed-int(300*cmd[0]))
        motor2.set_speed(-controlspeed+int(300*cmd[0]))
        sleep_us(5000-(ticks_us()-start))
    # stop and turn off motors
    motor1.set_speed(0)
    motor2.set_speed(0)
    motor1.set_off()
    motor2.set_off()

print ('start')
count = 0

while count < 5000 and BOOT_sw.value() == 1:
    count = count + 1
    tim.init(freq=10000, mode=Timer.PERIODIC, callback=step_cb) 
    balance()
    tim.deinit()
    led.value(not led.value())
    angle  = imu.pitch()
    rate   = imu.get_gy()
    motor1speed = motor1.get_speed()  
    motor2speed = motor2.get_speed()    
    print('angle = ',angle,'rate = ',rate,'motor1speed = ', motor1speed, 'motor2speed = ', motor2speed)

print ('exit')

motor1.set_speed(0)
motor1.set_off()
motor2.set_speed(0)
motor2.set_off()
#timer.deinit()
tim.deinit()

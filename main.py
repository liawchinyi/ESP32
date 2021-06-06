# UPYBBOT - micropython balancing robot
# cd C:\Python39\Lib\site-packages
# https://microcontrollerslab.com/micropython-mpu-6050-esp32-esp8266/
# C:\Python39\Lib\site-packages>esptool.py erase_flash
# C:\Python39\Lib\site-packages>esptool.py --port COM3 write_flash 0x1000 C:\ESP32a\esp32-idf3-20210202-v1.14.bin
# Complete project details at https://RandomNerdTutorials.com
# https://docs.micropython.org/en/latest/esp32/quickref.html
# https://randomnerdtutorials.com/micropython-wi-fi-manager-esp32-esp8266/
# http://micropython.org/webrepl/

# Complete project details at https://RandomNerdTutorials.com
import socket
import sys
import time
from time import ticks_us, ticks_cpu, sleep, sleep_us, sleep_ms
import machine
from machine import I2C, Pin, Timer, sleep, PWM

machine.freq(240000000)

addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)

print('listening on', addr)

pins = [machine.Pin(i, machine.Pin.IN) for i in (0, 2, 4, 5, 12, 13, 14, 15)]

html = """<!DOCTYPE html>
<html>
    <head> <title>ESP32 Pins</title> </head>
    <body> <h1>ESP32 Pins</h1>
        <table border="1"> <tr><th>Pin</th><th>Value</th></tr> %s </table>
        <script type="text/javascript">
            document.body.innerHTML = '';
        </script>
    </body>
</html>
"""

# set up stepper motors
from nemastepper import Stepper
motor1 = Stepper(26,25,12) #nemastepper
motor2 = Stepper(33,32,14) #nemastepper

led = Pin(19, Pin.OUT) #PWM(Pin(19), freq=1, duty=512) # create and configure in one go

BOOT_sw = Pin(0, Pin.IN, Pin.PULL_UP) # 输入

if BOOT_sw.value() == 0: # Press to terminate program
    print ('program terminated')
    sys.exit()

from mpu6050 import mpu6050
imu = mpu6050()

MAX_VEL = 2500 # 2000 usteps/sec = 500steps/sec = 2.5rps = 150rpm
MAX_ANGLE = 25  # degrees of tilt for speed control

def constrain(val,minv,maxv):
    if val<minv:
        return minv
    elif val>maxv:
        return maxv
    else:
        return val

# Complementary Filter A = rt/(rt + dt) where rt is response time, dt = period
def compf(fangle,accel,gyro,looptime,A):
    fangle = A * (fangle + gyro * looptime/1000) + (1-A) * accel
    return fangle

#speed P controiller - input is target speed, output is inclination angle
KpS = 0.01
def speedcontrol(target,current):
    global KpS
    error = target - current
    output = KpS * error
    return constrain(output,-MAX_ANGLE,+MAX_ANGLE)

angle = 0.0
rate = 0.0
weight = 0.9
gangle = 0.0
controlspeed = 0
fspeed = 0
delta = 0
tangle = 0
motor1speed = 0
motor2speed = 0
#stability PD controiller - input is target angle, output is acceleration
K = 6 # 7
Kp = 5.0 # 4
Kd = 0.9 # 0.4
Ki = 0.5
errorI = 0
def stability(target,current,rate):
    global K,Kp,Kd,errorI
    error = target - current
    errorI = errorI + error
    if errorI < -10 :
        errorI = -9
    if errorI > 10 :
        errorI = 9 
    output = Kp * error - Kd*rate #+ Ki*errorI
    output = int(K*output)
    if output < -2000 :
        output = -1999
    if output > 2000 :
        output = 1999   
    return output

#main balance loop runs every 5ms
def balance(self):
    global motor1, motor2, imu, angle, rate, motor2speed, weight
    global gangle, controlspeed, fspeed, delta, tangle, motor1speed
    start = time.ticks_ms()
    angle  = imu.pitch() - 5
    rate   = imu.get_gy() + 29
    weight  = imu.weight()
    #gangle = weight * (gangle + rate * (time.ticks_ms()-start)/1000) + (1-weight) * angle
    gangle = 0.99 * (gangle + rate * (time.ticks_ms()-start)/1000) + (1-0.99) * angle

    if abs(gangle) < 35 and BOOT_sw.value() == 1:  # give up if inclination angle >=45 degrees
        start = time.ticks_ms()
        # speed control
        motor1speed = motor1.get_speed()
        motor2speed = motor2.get_speed()
        actualspeed = (motor1speed+motor2speed)/2
        fspeed = 0.95 * fspeed + 0.05 * actualspeed
        cmd = [0,0] #radio.poll() # cmd[0] is turn speed, cmd[1] is fwd/rev speed
        tangle = speedcontrol(100*cmd[1],fspeed)
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

delay_start = time.ticks_ms()
print_start = time.ticks_ms()

#initializing the timer
#timer=Timer(8)
#timer.init(freq=1, mode=Timer.PERIODIC, callback=web_update)   

print ('start')

cl, addr = s.accept()
print('client connected from', addr)
cl_file = cl.makefile('rwb', 0)

while BOOT_sw.value() == 1 :

    if (time.ticks_ms()-delay_start) > 4 :
        balance(1)
        delay_start = time.ticks_ms()

    if (time.ticks_ms()-print_start) > 200 :
        print('TA',tangle,'GA',gangle,'A',angle,'R',rate,'W', weight, 'D',delta,'S1',motor1speed,'S2',motor2speed,'FS',fspeed)
        P = [0,1,2,3,4,5,6,7,8]
        A = ['Target Angle','G Angle','Angle','Rate','Weight','Delta','Motor1Speed','Motor2Speed','FS']
        B = [tangle,gangle,angle,rate,weight,delta,motor1speed,motor2speed,fspeed]
        rows = ['<tr><td>%s</td><td>%f</td></tr>' % (A[p], B[p]) for p in P]
        print_start = time.ticks_ms()

    #rows = ['<tr><td>%s</td><td>%d</td></tr>' % (str(p), p.value()) for p in pins]
    
    response = html % '\n'.join(rows)
    cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    cl.send(response)

cl.close()

print ('exit')

motor1.set_speed(0)
motor1.set_off()
motor2.set_speed(0)
motor2.set_off()
led.deinit()
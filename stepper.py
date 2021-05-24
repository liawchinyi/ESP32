
"""


Micropython步进电机控制库 


  控制思路就是在run()函数中不断的比较系统运行时间，如果时间到了就输出一个脉冲并计算下一次脉冲输出的时间，这个过程会一直循环执行，直到执行完毕。


  这段程序通过Stepper()类可以实例化多个步进电机，因而支持多台步进电机同时控制。


  这段程序初始化完成之后，正常运行时只要保证尽可能频繁的执行run()函数即可，而run()函数的执行时间非常小，所以程序中还可以集成一些其他耗时短的检测功能和控制功能，可以极大地丰富系统功能。


  这段程序控制步进电机的加减速过程是梯形的。


  搭配Arduino UNO硬件直接使用原生的Arduino程序可以同时控制5个步进电机同时实现加减速运行，启动过程平滑，丝毫没有卡顿现象。
  在ESP8266平台上，烧录MicroPython固件，使用上面的步进电机控制程序，最多只能控制3台步进电机同时运行，转速稍高时会出现卡顿现象，我估计是ESP8266+Micropython的代码执行速度跟不上导致的，如果有条件可以试一下ESP32。


————————————————


版权声明：本文为CSDN博主「Louistinda」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。


原文链接：https://blog.csdn.net/Lingdongtianxia/article/details/79633392
"""
import esp32
from time import ticks_us,ticks_cpu
import time
from machine import Pin

class Stepper():
  def __init__(self,stepPin,directionPin,enablePin):
    self.stepPin = Pin(stepPin,Pin.OUT)
    self.directionPin = Pin(directionPin,Pin.OUT)
    self.enablePin = Pin(enablePin,Pin.OUT)
    self.done = True
    self.debugModeFlag = False
    self.stepsToGo = 0
    self.stepsGone = 0
    self.acceleration = 0.00
    self.stepTime = 0.00
    self.starttime = 0.00
    self.stage1 = 0.00
    self.stage2 = 0.00
    self.stage1StepTime = 0.00
    self.stage3StepTime = 0.00

    self.nextStepTime = 0.00
    self.now_time = 0.00
    self.nextTimestamp = 0.00
    
  def startup(self):
    self.enablePin.value(0)
  def shutdown(self):
    self.enablePin.value(1)
  def stop(self):
    self.stepsToGo = 0
    self.done = True
  def isDown(self):
    return self.done
    
  def stepsGone(self):
    return self.stepsGone

  def rotate_init(self,speed,steps,acceleration,accelerationstep): #旋转参数初始化函数
    if (speed != 0 and steps != 0):
      if (steps > 0):
        self.directionPin.value(1)
      else:
        self.directionPin.value(0)
      self.done = False
      self.stepsToGo = abs(steps)
      self.acceleration = abs(acceleration)
      if (self.stepsToGo <= 4):
        self.acceleration = 0
      self.stepTime = 1000.0 * 1000.0 / abs(speed) #us
      self.startTime = self.stepTime * (self.acceleration + 1)
      self.stage1 = accelerationstep
      self.stage1StepTime = self.acceleration * self.stepTime / self.stage1
      self.stage2 = self.stepsToGo - accelerationstep
      self.stage3StepTime = self.acceleration * self.stepTime / (self.stepsToGo - self.stage2)
      self.stepsGone = 0
      self.now_time = ticks_us()
      self.step()

  def step(self):
    if (self.stepsToGo > self.stepsGone):
      self.stepPin.value(1)
      time.sleep_us(1)
      self.stepPin.value(0)
      self.stepsGone += 1
      self.nextStepTime = self.stepTime
      if (self.acceleration != 0):
        if (self.stepsGone < self.stage1):
          self.nextStepTime = self.startTime - self.stage1StepTime * self.stepsGone
        elif (self.stepsGone >= self.stage2):
          self.nextStepTime = self.stepTime + self.stage3StepTime * (self.stepsGone - self.stage2 + 1)
      self.nextTimestamp = ticks_us() + self.nextStepTime


      return self.stepsGone


    else:


      self.done = True





  def run(self): #尽可能频繁地执行run方法电机才会转


    if (not self.done and ticks_us() >= self.nextTimestamp):


      self.step()


      return self.stepsGone


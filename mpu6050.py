import machine
import os
from struct import unpack as unp
from math import atan2,degrees,pi
from machine import I2C, Pin, Timer, sleep
'''
def ar(i):
    switcher={
            0: 0x00,
            1: 0x08,
            2: 0x10,
            3: 0x18
            }
    return switcher.get(i,"Invalid")
'''
class mpu6050():
    '''
    Module for the MPY6050 6DOF IMU. 
    By default interrupts are disabled while reading or writing to the device. This
    prevents occasional bus lockups in the presence of pin interrupts, at the cost
    of disabling interrupts for about 250uS.
    '''
    mpu_addr = 0x68  # address of MPU6050
    _I2Cerror = "I2C communication failure"

    def __init__(self, side=1, disable_interrupts=False):
        addr=0x68
        # create i2c object
        self._timeout = 10
        self.disable_interrupts = False
        self.i2c = I2C(scl = Pin(22), sda = Pin(21), freq = 100000)   
        self.i2c.scan()
        self.addr = addr
        self.chip_id = int(unp('>h', self._read(14, 0x75, self.mpu_addr))[0])
        print('chip_id', self.chip_id)
        self.i2c.start()
        self.i2c.writeto(self.addr, bytearray([107, 0]))
        self.i2c.stop()

        # now apply user setting for interrupts
        self.disable_interrupts = disable_interrupts        
        # wake it up
        self.wake()
        self.accel_range(1)
        self._ar = self.accel_range()
        self.gyro_range(0)
        self._gr = self.gyro_range()

    # wake
    def wake(self):
        '''
        Wakes the device.
        '''
        try:
            self._write(bytearray(0x01), 0x6B, self.mpu_addr)
        except OSError:
            print(accel._I2Cerror)
        return 'awake'

    # read from device
    def _read(self, count, memaddr, devaddr):
        '''
        Perform a memory read. Caller should trap OSError. Possible values of
        error args[0]: errorno.ETIMEDOUT errno.EBUSY or errno.EIO
        '''
        irq_state = True
        if self.disable_interrupts:
            irq_state = machine.disable_irq()
        self.i2c.start()
        result = self.i2c.readfrom_mem(devaddr,memaddr,count)
        self.i2c.stop()                                
        machine.enable_irq(irq_state)
        return result

    # write to device
    def _write(self, data, memaddr, devaddr):
        '''
        Perform a memory write. Caller should trap OSError.
        '''
        irq_state = True
        if self.disable_interrupts:
            irq_state = machine.disable_irq()
        self.i2c.start()    
        result = self.i2c.writeto_mem(devaddr,memaddr,data)
        self.i2c.stop()                                   
        machine.enable_irq(irq_state)
        return result

    # accelerometer range
    def accel_range(self, accel_range=None):
        '''
        Returns the accelerometer range or sets it to the passed arg.
        Pass:               0   1   2   3
        for range +/-:      2   4   8   16  g 
        '''
        # set range
        try:
            if accel_range is None:
                pass
            else:
                arl = (0x00, 0x08, 0x10, 0x18)                
                ar = arl[accel_range]
                try:
                    self._write(bytearray(ar), 0x1C, self.addr)
                except IndexError:
                    print('accel_range can only be 0, 1, 2 or 3')
            # get range
            ari = int(unp('<H', self._read(14, 0x1C, self.addr))[0]/8)
        except OSError:
            ari = None
        if ari is not None:
            self._ar = ari
        return ari

    # gyroscope range
    def gyro_range(self, gyro_range=None):
        '''
        Returns the gyroscope range or sets it to the passed arg.
        Pass:               0   1   2    3
        for range +/-:      250 500 1000 2000  degrees/second
        '''
        # set range
        try:
            if gyro_range is None:
                pass
            else:
                grl = (0x00, 0x08, 0x10, 0x18)
                gr = grl[gyro_range]
                try:
                    self._write(bytearray(gr), 0x1B, self.addr)              
                except IndexError:
                    print('gyro_range can only be 0, 1, 2 or 3')
            # get range
            gri = int(unp('<H', self._read(14, 0x1B, self.addr))[0]/8)
        except OSError:
            gri = None

        if gri is not None:
            self._gr = gri
        return gri

    def get_raw_values(self):
        self.i2c.start()
        a = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        self.i2c.stop()
        return a

    def get_ints(self):
        b = self.get_raw_values()
        c = []
        for i in b:
            c.append(i)
        return c

    def bytes_toint(self, firstbyte, secondbyte):
        if not firstbyte & 0x80:
            return firstbyte << 8 | secondbyte
        return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

    def get_values(self):
        raw_ints = self.get_raw_values()
        vals = {}
        vals["AcX"] = self.bytes_toint(raw_ints[0], raw_ints[1])
        vals["AcY"] = self.bytes_toint(raw_ints[2], raw_ints[3])
        vals["AcZ"] = self.bytes_toint(raw_ints[4], raw_ints[5])
        vals["Tmp"] = self.bytes_toint(raw_ints[6], raw_ints[7]) / 340.00 + 36.53
        vals["GyX"] = self.bytes_toint(raw_ints[8], raw_ints[9])
        vals["GyY"] = self.bytes_toint(raw_ints[10], raw_ints[11])
        vals["GyZ"] = self.bytes_toint(raw_ints[12], raw_ints[13])
        return vals  # returned in range of Int16
        # -32768 to 32767

    def val_test(self):  # ONLY FOR TESTING! Also, fast reading sometimes crashes IIC
        from time import sleep
        while 1:
            print(self.get_values())
            sleep(0.05)

    # get raw acceleration
    def get_accel_raw(self):
        '''
        Returns the accelerations on xyz in bytes.
        '''
        try:
            axyz = self._read(6, 0x3B, self.mpu_addr)
        except OSError:
            axyz = b'\x00\x00\x00\x00\x00\x00'
        return axyz

    # get pitch  
    def pitch(self):
        '''
        Returns pitch angle in degrees based on x and c accelerations.
        
        '''
        scale = (16384, 8192, 4096, 2048)
        raw = self.get_accel_raw()
        x = unp('>h', raw[0:2])[0]/scale[self._ar]
        z = unp('>h', raw[4:6])[0]/scale[self._ar]
        pitch = degrees(pi+atan2(-x,-z))
        if (pitch>=180) and (pitch<=360):
            pitch-=360
        return -pitch

    # get raw gyro
    def get_gyro_raw(self):
        '''
        Returns the turn rate on xyz in bytes.
        '''
        try:
            gxyz = self._read(6, 0x43, self.mpu_addr)
        except OSError:
            gxyz = b'\x00\x00\x00\x00\x00\x00'
        return gxyz        

    # get gyro pitch - y - axis in degrees
    def get_gy(self):
        scale = (131.0, 65.5, 32.8, 16.4)
        raw = self.get_gyro_raw()
        gy =  unp('>h', raw[2:4])[0]/scale[self._gr]
        return gy    
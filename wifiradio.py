# WiFiradio.py -- get radio buttons 

import machine
from machine import UART
import struct

class WiFiRadio():

    def __init__(self, uart_id):
        self.uart = UART(uart_id, 115200)
        self.nt = {"/1/turn":0,
                   "/1/speed":1,
                   "/2/left":0,
                   "/2/right":0,
                   "/2/forward":1,
                   "/2/reverse":1
                   }
        self._state = [0.0, 0.0]
        s = self._read_prologue()
        self._ip = self._extract_ipaddr(s)
        self._period = 0 

    def getipaddr(self):
        return self._ip

    def _extract_ipaddr(self,s):
        if len(s)<20:           #missed initial output containing ip address?
            return ''
        si = s.find('192')
        if si<0:
            return ''
        ei = s[si:].find('\n')
        if si<0:
            return ''
        return s[si:ei+si]
        
    def _read_prologue(self):
        s = ''
        ch = self.uart.read()
        if (ch<0):
            return s
        while chr(ch)!='#':
            s = s + chr(ch)
            ch = self.uart.read()
            if(ch<0):
                return s
        return s

    def guardband(self, val):
        if val<0.05 and val > -0.05:
            return  0.0
        else:
            return  val
      
    def update(self, name,val):
        try:
            id = self.nt[name]
            self._state[id] = self.guardband(val)
        except KeyError:
            pass

    # process incoming messages from esp8266 and update state
    # message format is <#> <size> <simple OSC message> </n>
    def poll(self):
        self._period = (self._period+1) % 4
        if self._period == 0:
            return self._state         
        start = False
        while self.uart.any() and not start:
            ch = self.uart.read()
            start = (chr(ch) == '#')   # start of message
        if not start:
            return self._state
        nb = self.uart.read()      # size of message
        name =''
        ch = self.uart.read()
        while ch!=0:
            name += chr(ch)
            ch = self.uart.read()
        if len(name)<=4:
            return self._state
        while ch==0:
            ch = self.uart.read()
        for j in range(3):
            ch = self.uart.read()  # skip format string
        buf = bytearray(4)
        for j in range(4):
            buf[j] = self.uart.read()
        fval = struct.unpack('>f',buf)
        self.update(name, fval[0])
        ch = self.uart.read()    # read terminating linefeed
        return self._state
     
      






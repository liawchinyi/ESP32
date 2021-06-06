import network
sta_if = network.WLAN(network.STA_IF)
if not sta_if.isconnected():
    print('connecting to network...')
    sta_if.active(True)
    sta_if.connect('SINGTEL-5731', '4637667190')
    while not sta_if.isconnected():
        pass
print('network config:', sta_if.ifconfig())

# This file is executed on every boot (including wake-boot from deepsleep)
import esp
#esp.osdebug(None)
#http://micropython.org/webrepl/
import webrepl
webrepl.start()

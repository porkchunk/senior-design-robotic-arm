from machine import Pin, I2C
import TSL2591
from time import sleep


addr = 0x29
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)

tsl = TSL2591.TSL2591(i2c, addr)


while True:
#     visible = tsl.visible
#     print(visible)
    lux = tsl.lux
    print(int(lux))
    sleep(0.3)
import machine
import time

uart0 = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1))

while True:
    
    while uart0.any():
        uart0.read(1)
    
    uart0.write("a".encode())
    if uart0.any():
        print(int.from_bytes(uart0.read(5),'big'))
    time.sleep(0.01)
    
    uart0.write("b".encode())
    if uart0.any():
        print(uart0.read(5))
    time.sleep(0.01)
from machine import Pin, PWM, ADC
from time import ticks_us

direction_motor1 = Pin(8, Pin.OUT)
direction_motor2 = 0

direction_motor4 = Pin(6, Pin.OUT)
direction_motor5 = Pin(5, Pin.OUT)
direction_motor6 = Pin(4, Pin.OUT)

button_left = Pin(14, Pin.IN, Pin.PULL_DOWN)
button_right = Pin(15, Pin.IN, Pin.PULL_DOWN)

motor1 = PWM(Pin(21), freq=50)
motor2 = PWM(Pin(20), freq=50)
motor4 = PWM(Pin(18), freq=50)
motor5 = PWM(Pin(16), freq=50)
motor6 = PWM(Pin(17), freq=50)
claw = PWM(Pin(22), freq=50)

motor_move = ADC(Pin(26))
claw_change = ADC(Pin(27))

global last_current_time
last_current_time = 0
global current_motor
current_motor = 0

def button_handler(pin):
    global last_current_time
    global current_motor
    interrupt_time = ticks_us()/1e6
    if((interrupt_time - last_current_time) > 0.2):
        current_motor = ((current_motor + 1) % 6)
        if(current_motor == 2):
            current_motor = 3
        last_interrupt_time = interrupt_time
    
button_right.irq(trigger=Pin.IRQ_FALLING, handler = button_handler)

def all_motors_off():
    motor1.duty_u16(0)
    motor2.duty_u16(0)
    motor4.duty_u16(0)
    motor5.duty_u16(0)
    motor6.duty_u16(0)
    

def move_motor1():
    #all_motors_off()
    if(motor_move.read_u16() > 45000):
        direction_motor1.on()
        motor1.duty_u16(int((0.4)*65535))
    elif(motor_move.read_u16() < 25000):
        direction_motor1.off()
        motor1.duty_u16(int((0.4)*65535))
    else:
        motor1.duty_u16(0)

def move_motor2():
    #all_motors_off()
    if(motor_move.read_u16() > 45000):
        motor2.duty_u16(int((1.9/20)*65535))
    elif(motor_move.read_u16() < 25000):
        motor2.duty_u16(int((1.3/20)*65535))
    else:
        motor2.duty_u16(0)

def move_motor4():
    #all_motors_off()
    if(motor_move.read_u16() > 45000):
        direction_motor4.on()
        motor4.duty_u16(int((0.45)*65535))
    elif(motor_move.read_u16() < 25000):
        direction_motor4.off()
        motor4.duty_u16(int((0.45)*65535))
    else:
        motor4.duty_u16(0)
        
def move_motor5():
    #all_motors_off()
    if(motor_move.read_u16() > 45000):
        direction_motor5.on()
        motor5.duty_u16(int((0.35)*65535))
    elif(motor_move.read_u16() < 25000):
        direction_motor5.off()
        motor5.duty_u16(int((0.27)*65535))
    else:
        motor5.duty_u16(0)
        
def move_motor6():
    #all_motors_off()
    if(motor_move.read_u16() > 45000):
        direction_motor6.on()
        motor6.duty_u16(int((0.7)*65535))
    elif(motor_move.read_u16() < 25000):
        direction_motor6.off()
        motor6.duty_u16(int((0.7)*65535))
    else:
        motor6.duty_u16(0)
        
def joystick_claw():
    if(claw_change.read_u16() > 45000):
        claw.duty_u16(int((1.8/20)*65535))
    elif(claw_change.read_u16() < 25000):
        claw.duty_u16(int((1.4/20)*65535))

        
while True:
    if(current_motor == 0):
        move_motor1()
    if(current_motor == 1):
        move_motor2()
    if(current_motor == 3):
        move_motor4()
    if(current_motor == 4):
        move_motor5()
    if(current_motor == 5):
        move_motor6()
        
    joystick_claw()
    
from machine import Pin, PWM, ADC
from time import sleep
from ulab import numpy as np
import math
from robot import motor_move, forward_kinematics, jacobian, robot_move, claw_move

automatic_manual = Pin(15, Pin.IN, Pin.PULL_DOWN)

pwm1 = PWM(Pin(21))
pwm2 = PWM(Pin(20))
pwm3 = PWM(Pin(19))
pwm4 = PWM(Pin(18))
pwm5 = PWM(Pin(17))

pwm1.freq(50)
pwm2.freq(50)
pwm3.freq(50)
pwm4.freq(50)
pwm5.freq(50)
ADC_valueY = ADC(26)
ADC_valueZ = ADC(27)
ADC_valueX = ADC(28)

pin = Pin(9, Pin.IN, Pin.PULL_DOWN)

STARTING_X = 25.6
STARTING_Y = 0
STARTING_Z = 19.9

motor_move(0,np.pi/2,-np.pi/2,0,1);

while True:
    robot_move(STARTING_X + 0.3, STARTING_Y + 0.3, STARTING_Z + 0.3, 0, 1)
# while True:
#     if(automatic_manual.value() == 1):
#         motor_move(0,np.pi/2,-np.pi/2,0,1)
# 
#         sleep(2)
# 
#         motor_move(0,np.pi/3,-np.pi/2,0,1)
#     else:
#         motor_move(0,np.pi/2,-np.pi/2,0,1)
# 
#         sleep(2)
# 
#         motor_move(0,np.pi,-np.pi/2,0,1)

# motor_move(0,np.pi,-np.pi/2,0,1)
# sleep(2)
# countZ = 0
# countY = 0
# countX = 0
# countRotate = 0
# position = 0
# STARTING_X = 25.6
# STARTING_Y = 0
# STARTING_Z = 19.9
# STARTING_PITCH = 0
# claw_move(0)
# while True:
#     if ADC_valueZ.read_u16() > 36000:
#         countZ = countZ - 0.4
#         print(ADC_valueZ.read_u16())
#     if ADC_valueZ.read_u16() < 24000:
#         countZ = countZ + 0.4
#         print(ADC_valueZ.read_u16())
#     if ADC_valueY.read_u16() > 36000:
#         #countY = countY + 0.4
#      #   print(ADC_valueY.read_u16())
#         countRotate = countRotate + 0.012
#     if ADC_valueY.read_u16() < 24000:
#         countRotate = countRotate - 0.012
#         #countY = countY - 0.4
#       #  print(ADC_valueY.read_u16())
#     if ADC_valueX.read_u16() < 1000:
#       countX = countX - 0.4
#     if ADC_valueX.read_u16() > 3500:
#       countX = countX + 0.4
#     if pin.value() == 0:
#         position = (position + 1)%2
#         claw_move(position)
#         while pin.value() == 0:
#             pass
#     
# #     if(1.5 + countRotate > 2.5):
# #         countRotate = countRotate - 0.012
# #     elif(1.5 + countRotate < 0.5):
# #         countRotate = countRotate + 0.012
#         
#     if math.sqrt(math.pow(STARTING_X + countX,2) + math.pow(STARTING_Y,2) + math.pow(STARTING_Z + countZ -9.1,2)) >= 35 or STARTING_X + countX < 0 or STARTING_Z + countZ < 0:
#         countX = countX/1.4
#         countY = countY/1.4
#         countZ = countZ/1.4
#         robot_move(STARTING_X + countX, STARTING_Y, STARTING_Z + countZ, STARTING_PITCH)
#         sleep(3)
#         
# 
#     pwm1.duty_u16(int(65025*(1.5+countRotate)/20))
#     robot_move(STARTING_X + countX, STARTING_Y, STARTING_Z + countZ, STARTING_PITCH)
    
   
        
    
# sleep(2)
# for i in range(200):
#     robot_move(25,-10*i/200,20,0)
# 
# sleep(1)
# for i in range(400):
#     robot_move(25,(10*i/200)-10,20,0)



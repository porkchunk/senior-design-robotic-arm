from ulab import numpy as np
import math
from machine import PWM, Pin
from time import sleep, ticks_us

distance_link_1 = 9.5 
distance_link_2 = 10.4 
distance_link_3 = 8.9 
distance_link_4 = 16.7 

global signal1,signal2,signal3,signal4,claw_position

pwm1 = PWM(Pin(18))
pwm2 = PWM(Pin(20))
pwm3 = PWM(Pin(21))
pwm4 = PWM(Pin(19))
pwm5 = PWM(Pin(17))

def transform_matrix(alpha, a, d, theta):
    time1 = ticks_us()
    ctheta = math.cos(theta)
    calpha = math.cos(alpha)
    stheta = math.sin(theta)
    salpha = math.sin(alpha)
    
    T1 = np.array([ctheta, -stheta, 0, a])
    T2 = np.array([stheta*calpha, ctheta*calpha, -salpha, -d*salpha])
    T3 = np.array([stheta*salpha, ctheta*salpha, calpha, d*calpha])
    T4 = np.array([0, 0, 0, 1])
    T = np.array([T1,
                  T2,
                  T3,
                  T4])
    time2 = ticks_us()
    print((time2 - time1))
    return T

def forward_kinematics(theta1,theta2,theta3,theta4):
    time1 = ticks_us()
    alpha = np.array([0, (np.pi)/2, 0, 0, 0])
    a = np.array([0, 0, distance_link_2, distance_link_3, distance_link_4])
    d = np.array([distance_link_1, 0, 0, 0, 0])
    Q = np.array([theta1, theta2, theta3, theta4, 0])
    
    T01 = transform_matrix(alpha[0],a[0],d[0],Q[0])
    T12 = transform_matrix(alpha[1],a[1],d[1],Q[1])
    T23 = transform_matrix(alpha[2],a[2],d[2],Q[2])
    T34 = transform_matrix(alpha[3],a[3],d[3],Q[3])
    T45 = transform_matrix(alpha[4],a[4],d[4],Q[4])
    
    T02 = np.dot(T01,T12)
    T24 = np.dot(T23,T34)
    T25 = np.dot(T24,T45)
    T05 = np.dot(T02,T25)
    
    X = T05[0][3]
    Y = T05[1][3]
    Z = T05[2][3]
    PITCH = math.atan(-T05[2][0]/(math.sqrt(math.pow(T05[0][0],2) + math.pow(T05[1][0],2))))
    
    XYZPITCH = np.array([[X],[Y],[Z],[PITCH]])
    time2 = ticks_us()
    print(time2 - time1)
    return XYZPITCH

def jacobian(theta1,theta2,theta3,theta4):
    time1 = ticks_us()
    s1 = math.sin(theta1)
    s2 = math.sin(theta2)
    s3 = math.sin(theta3)
    s4 = math.sin(theta4)
    c1 = math.cos(theta1)
    c2 = math.cos(theta2)
    c3 = math.cos(theta3)
    c4 = math.cos(theta4)

    J11 = (-s1*c2)*(distance_link_4*c3*c4 - distance_link_4*s3*s4 + distance_link_3*c3 + distance_link_2) + (s1*s2)*(distance_link_4*s3*c4 + distance_link_4*c3*s4 + distance_link_3*s3)
    J12 = (-s2*c1)*(distance_link_4*c3*c4 - distance_link_4*s3*s4 + distance_link_3*c3 + distance_link_2) - (c1*c2)*(distance_link_4*s3*c4 + distance_link_4*c3*s4 + distance_link_3*s3)
    J13 = (c1*c2)*(-distance_link_4*s3*c4 - distance_link_4*c3*s4 - distance_link_3*s3) + (-c1*s2)*(distance_link_4*c3*c4 - distance_link_4*s3*s4 + distance_link_3*c3)
    J14 = (c1*c2)*(-distance_link_4*c3*s4 - distance_link_4*s3*c4) + (-c1*s2)*(-distance_link_4*s3*s4 + distance_link_4*c3*c4)
    J21 = (c1*c2)*(distance_link_4*c3*c4 - distance_link_4*s3*s4 + distance_link_3*c3 + distance_link_2) - (c1*s2)*(distance_link_4*s3*c4 + distance_link_4*c3*s4 + distance_link_3*s3)
    J22 = (-s1*s2)*(distance_link_4*c3*c4 - distance_link_4*s3*s4 + distance_link_3*c3 + distance_link_2) - (s1*c2)*(distance_link_4*s3*c4 + distance_link_4*c3*s4 + distance_link_3*s3)
    J23 = (s1*c2)*(-distance_link_4*s3*c4 - distance_link_4*c3*s4 - distance_link_3*s3) - (s1*s2)*(distance_link_4*c3*c4 - distance_link_4*s3*s4 + distance_link_3*c3)
    J24 = (s1*c2)*(-distance_link_4*c3*s4 - distance_link_4*s3*c4) - (s1*s2)*(-distance_link_4*s3*s4 + distance_link_4*c3*c4)
    J31 = 0
    J32 = c2*(distance_link_4*c3*c4 - distance_link_4*s3*s4 + distance_link_3*c3 + distance_link_2) - s2*(distance_link_4*s3*c4 + distance_link_4*c3*s4 + distance_link_3*s3)
    J33 = s2*(-distance_link_4*s3*c4 - distance_link_4*c3*s4 - distance_link_3*s3) + c2*(distance_link_4*c3*c4 - distance_link_4*s3*s4 + distance_link_3*c3)
    J34 = s2*(-distance_link_4*c3*s4 - distance_link_4*s3*c4) + c2*(-distance_link_4*s3*s4 + distance_link_4*c3*c4)
    
    J41 = 0
    J42 = -c1
    J43 = -c1
    J44 = -c1

    J = np.array([[J11,J12,J13,J14],[J21,J22,J23,J24],[J31,J32,J33,J34],[J41,J42,J43,J44]])
    time2 = ticks_us()
    print(time2-time1)
    return J

def map_function(input_number,input_start,input_end,output_start,output_end):
    slope = (output_end - output_start) / (input_end - input_start)
    output = output_start + slope * (input_number - input_start)
    
    return output

def motor_move(theta1,theta2,theta3,theta4,config):
    '''
    config = 0 is auto mode and manual xyz control
    config = 1 is manual mode with rotation and xy control
    '''
    global signal1,signal2,signal3,signal4
   
    signal1 = (map_function(theta1,-np.pi/2,np.pi/2,0.5,2.5)/20)*65535
    signal2 = (map_function(theta2,np.pi,0,0.5,2.5)/20)*65535
    signal3 = (map_function(theta3,(-3*np.pi)/4,0,0.56,2.06)/20)*65535
    signal4 = (map_function(theta4,-np.pi/2,np.pi/2,0.5,2.5)/20)*65535
    
    if(config == 1):
        pwm1.duty_u16(int(signal1))
    pwm2.duty_u16(int(signal2))
    pwm3.duty_u16(int(signal3))
    pwm4.duty_u16(int(signal4))

def robot_move(x,y,z,pitch,config):
    '''
    config = 0 is auto mode
    config = 1 is any manual mode
    '''
    global signal1,signal2,signal3,signal4
    x_compensation = 0
    y_compensation = 0
    z_compensation = 0
    
    x = x + x_compensation
    y = y + y_compensation
    z = z + z_compensation
    
    if(config == 0):
        if math.sqrt(math.pow(x,2) + math.pow(y,2) + math.pow(z-9.1,2)) >= 35 or x < 0 or z < 0:
            print("x,y,z out of bounds")
            return
    
    theta1 = map_function(20*signal1/65535,0.5,2.5,-np.pi/2,np.pi/2)
    theta2 = map_function(20*signal2/65535,0.5,2.5,np.pi,0)
    theta3 = map_function(20*signal3/65535,0.56,2.06,(-3*np.pi)/4,0)
    theta4 = map_function(20*signal4/65535,0.5,2.5,-np.pi/2,np.pi/2)
    
    x_final = np.array([[x],[y],[z],[pitch]])
    initial_angle = np.array([[theta1],[theta2],[theta3],[theta4]]) # [[0],[np.pi/5],[-np.pi/5],[0]]
    jacobian_matrix = jacobian(theta1,theta2,theta3,theta4) # 0,np.pi/5,-np.pi/5,0
    x_initial = forward_kinematics(theta1,theta2,theta3,theta4) # 0,np.pi/5,-np.pi/5,0
    x_difference = [1,1,1.1]
    total_distance = x_final - x_initial
    time_step = 0.01
    
    count = 0
    speed = 25
    
    while np.linalg.norm(x_difference) >= 0.1:
        x_difference = x_final - x_initial
        
        velocity = (x_difference/np.linalg.norm(x_difference))*speed
        
        lambda_damping = 1
        identity_matrix = np.eye(jacobian_matrix.shape[0])  # Use .shape[1] if jacobian_matrix is not square
        jacobian_inverse = np.dot(np.linalg.inv(np.dot(jacobian_matrix.T, jacobian_matrix) + lambda_damping * identity_matrix),(jacobian_matrix.T))
        #jacobian_inverse = np.linalg.inv(jacobian_matrix)
        delta_angle = np.dot(jacobian_inverse,velocity)
        
        new_angle = initial_angle + delta_angle*time_step
        initial_angle = new_angle
        
        motor_move(new_angle[0][0],new_angle[1][0],new_angle[2][0],new_angle[3][0],1)

        x_initial = forward_kinematics(new_angle[0][0],new_angle[1][0],new_angle[2][0],new_angle[3][0])
        jacobian_matrix = jacobian(new_angle[0][0],new_angle[1][0],new_angle[2][0],new_angle[3][0])
        
        count = count + 1
        if count >= (np.linalg.norm(total_distance)/(speed*time_step)): #count - 100 if doing auto control
            break
   
    return x_initial

def claw_move(position):
    global claw_position
    
    if(position == 0):
        pwm5.duty_u16(int((1.4/20)*65535)) #previous 1.4
    if(position == 1):
        pwm5.duty_u16(int((2.15/20)*65535)) #previous 2.15/20
        


        

        
    

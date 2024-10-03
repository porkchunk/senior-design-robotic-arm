from ulab import numpy as np
import math
from machine import PWM, Pin
from time import sleep, ticks_us

distance_link_1 = 9.5 
distance_link_2 = 10.4 
distance_link_3 = 8.9 
distance_link_4 = 16.7

def transform_matrix(alpha, a, d, theta):
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
    
    return T

def forward_kinematics(theta1,theta2,theta3,theta4):
    
    alpha = np.array([0, (np.pi)/2, 0, 0, 0])
    a = np.array([0, 0, distance_link_2, distance_link_3, distance_link_4])
    d = np.array([distance_link_1, 0, 0, 0, 0])
    Q = np.array([theta1, theta2, theta3, theta4, 0])
    
    T01 = transform_matrix(alpha[0],a[0],d[0],Q[0])
    T12 = transform_matrix(alpha[1],a[1],d[1],Q[1])
    T23 = transform_matrix(alpha[2],a[2],d[2],Q[2])
    T34 = transform_matrix(alpha[3],a[3],d[3],Q[3])
    T45 = transform_matrix(alpha[4],a[4],d[4],Q[4])
#     T56 = transform_matrix(alpha[4],a[4],d[4],Q[4])
#     T67 = transform_matrix(alpha[4],a[4],d[4],Q[4])
    
    T02 = np.dot(T01,T12)
    T24 = np.dot(T23,T34)
    T25 = np.dot(T24,T45)
    T05 = np.dot(T02,T25)
#     T57 = np.dot(T56,T57)
    
    X = T05[0][3]
    Y = T05[1][3]
    Z = T05[2][3]
    
    XYZ = np.array([[X],[Y],[Z]])
    
    return XYZ

def jacobian(theta, h=1e-4):
    n = len(theta)  
    f = forward_kinematics  
    pos = f(theta[0], theta[1], theta[2], theta[3]) #, theta[4], theta[5]
    m = len(pos)  
    
    J = np.zeros((m, n))
    time1 = ticks_us()
    for i in range(n):
        theta_perturbed = theta.copy()
        theta_perturbed[i] += h
        
        pos_perturbed = f(theta_perturbed[0], theta_perturbed[1], theta_perturbed[2], theta_perturbed[3]) #, theta_perturbed[4], theta_perturbed[5]
        
        dpos = (pos_perturbed - pos) / h
        
        for j in range(m):
            J[j, i] = dpos[j]
    time2 = ticks_us()
    print((time2 - time1)/1e3)
    return J

theta = np.array([0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
J = jacobian(theta)
print(J)
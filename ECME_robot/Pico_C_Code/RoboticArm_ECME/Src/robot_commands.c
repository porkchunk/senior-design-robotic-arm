#include "robot_commands.h"
#include "motor.h"
#include "pico/stdlib.h"
#include "main.h"
#include "SPI.h"
#include "robot_modes.h"
#include "pico/multicore.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float position[POSITION_SIZE];
float theta[6];
volatile bool claw_position;

//Unused but keeping for demonstration purposes
void transform_matrix(float alpha, float a, float d, float theta, float T[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]) {
    T[0][0] = cos(theta);
    T[0][1] = -sin(theta);
    T[0][2] = 0;
    T[0][3] = a;

    T[1][0] = sin(theta) * cos(alpha);
    T[1][1] = cos(theta) * cos(alpha);
    T[1][2] = -sin(alpha);
    T[1][3] = -d * sin(alpha);

    T[2][0] = sin(theta) * sin(alpha);
    T[2][1] = cos(theta) * sin(alpha);
    T[2][2] = cos(alpha);
    T[2][3] = d * cos(alpha);

    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;
}

void forward_kinematics(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6, float position[POSITION_SIZE], float rotation_matrix[3][3]){
    float c1 = cos(theta1);
    float c2 = cos(theta2);
    float c3 = cos(theta3);
    float c4 = cos(theta4);
    float c5 = cos(theta5);
    float c6 = cos(theta6);

    float s1 = sin(theta1);
    float s2 = sin(theta2);
    float s3 = sin(theta3);
    float s4 = sin(theta4);
    float s5 = sin(theta5);
    float s6 = sin(theta6);

    position[0] = (c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) - DISTANCE_LINK_7*(s1*s6 - c5*c6*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) + c6*s5*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4))) + c1*c2*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) - DISTANCE_LINK_3*c1*s2*s3;
    position[1] = DISTANCE_LINK_7*(c1*s6 - c6*s5*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)) + c5*c6*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3))) - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)) + (c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) + c2*s1*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) - DISTANCE_LINK_3*s1*s2*s3;
    position[2] = DISTANCE_LINK_1 + (c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) + DISTANCE_LINK_7*(c5*c6*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4)) + c6*s5*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3))) + s2*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) + (c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5) + DISTANCE_LINK_3*c2*s3;
 
    float R11 = c5*c6*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - s1*s6 - c6*s5*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4));
    float R21 = c1*s6 - c6*s5*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)) + c5*c6*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3));
    float R31 = c5*c6*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4)) + c6*s5*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3));

    float R12 = s5*s6*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) - c5*s6*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - c6*s1;
    float R22 = c1*c6 - c5*s6*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) + s5*s6*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4));
    float R32 = - c5*s6*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4)) - s5*s6*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3));

    float R13 = - c5*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) - s5*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3));
    float R23 = - c5*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)) - s5*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3));
    float R33 = c5*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)) - s5*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4));
    /*
    if(fabs(R11) < 0.000001){
        R11 = 0;
    }
    if(fabs(R21) < 0.000001){
        R21 = 0;
    }
    if(fabs(R31) < 0.000001){
        R31 = 0;
    }
    
    if(fabs(R12) < 0.000001){
        R12 = 0;
    }
    if(fabs(R22) < 0.000001){
        R22 = 0;
    }
    if(fabs(R32) < 0.000001){
        R32 = 0;
    }
    if(fabs(R13) < 0.000001){
        R13 = 0;
    }
    if(fabs(R23) < 0.000001){
        R23 = 0;
    }
    if(fabs(R33) < 0.000001){
        R33 = 0;
    }
    */
    position[3] = -asinf(R31); //PITCH
    position[4] = atanf(R21/cos(position[3])/R11/cos(position[3])); //YAW
    
    rotation_matrix[0][0] = R11;
    rotation_matrix[1][0] = R21;
    rotation_matrix[2][0] = R31;

    rotation_matrix[0][1] = R12;
    rotation_matrix[1][1] = R22;
    rotation_matrix[2][1] = R32;

    rotation_matrix[0][2] = R13;
    rotation_matrix[1][2] = R23;
    rotation_matrix[2][2] = R33;

    /* Unused section but will keep for demonstration purposes
    float alpha[7] = {0, M_PI/2, 0, 0, 0, -M_PI/2, 0};
    float a[7] = {0, 0, DISTANCE_LINK_2, DISTANCE_LINK_3, DISTANCE_LINK_4, DISTANCE_LINK_5, DISTANCE_LINK_6};
    float d[7] = {DISTANCE_LINK_1, 0, 0, 0, 0, 0, 0};
    float Q[7] = {theta1, theta2, theta3, theta4, theta5, theta6, 0};

    float T01[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T12[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T23[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T34[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T45[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T56[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T67[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    
    float T46[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T02[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T24[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T04[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T06[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T07[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];

    transform_matrix(alpha[0], a[0], d[0], Q[0], T01);
    transform_matrix(alpha[1], a[1], d[1], Q[1], T12);
    transform_matrix(alpha[2], a[2], d[2], Q[2], T23);
    transform_matrix(alpha[3], a[3], d[3], Q[3], T34);
    transform_matrix(alpha[4], a[4], d[4], Q[4], T45);
    transform_matrix(alpha[5], a[5], d[5], Q[5], T56);
    transform_matrix(alpha[6], a[6], d[6], Q[6], T67);

    multiply_matrices(T01, T12, T02); 
    multiply_matrices(T23, T34, T24); 
    multiply_matrices(T02, T24, T04); 
    multiply_matrices(T45, T56, T46); 
    multiply_matrices(T04, T46, T06); 
    multiply_matrices(T06, T67, T07);  
    
    position[0] = T07[0][3]; //X
    position[1] = T07[1][3]; //Y
    position[2] = T07[2][3]; //Z
    position[3] = atanf(T07[2][1]/T07[2][2]); //Roll
    position[4] = atanf(-T07[2][0]/(sqrtf(powf(T07[0][0],2) + powf(T07[1][0],2)))); //Pitch
    position[5] = atanf(T07[1][0]/T07[0][0]); //Yaw
    */
}

void jacobian_function(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6, float jacobian[6][6]){
    float c1 = cos(theta1);
    float c2 = cos(theta2);
    float c3 = cos(theta3);
    float c4 = cos(theta4);
    float c5 = cos(theta5);
    float c6 = cos(theta6);

    float s1 = sin(theta1);
    float s2 = sin(theta2);
    float s3 = sin(theta3);
    float s4 = sin(theta4);
    float s5 = sin(theta5);
    float s6 = sin(theta6);

    float J11 = (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)) - DISTANCE_LINK_7*(c1*s6 - c6*s5*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)) + c5*c6*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3))) - (c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - c2*s1*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) + DISTANCE_LINK_3*s1*s2*s3;
    float J21 = (c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) - DISTANCE_LINK_7*(s1*s6 - c5*c6*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) + c6*s5*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4))) + c1*c2*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) - DISTANCE_LINK_3*c1*s2*s3;
    uint J31 = 0;
    uint J41 = 0;
    uint J51 = 0;
    uint J61 = 1;

    float J12 = - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - DISTANCE_LINK_7*(c5*c6*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) + c6*s5*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3))) - (c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - c1*s2*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) - DISTANCE_LINK_3*c1*c2*s3;
    float J22 = - DISTANCE_LINK_7*(c6*s5*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) + c5*c6*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4))) - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) - (c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - s1*s2*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) - DISTANCE_LINK_3*c2*s1*s3;
    float J32 = (c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) + c2*(DISTANCE_LINK_2 + DISTANCE_LINK_3*c3) + DISTANCE_LINK_7*(c5*c6*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)) - c6*s5*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))) - (c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5) - DISTANCE_LINK_3*s2*s3;
    float J42 = s1;
    float J52 = -c1;
    uint  J62 = 0;


    float J13 = - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - DISTANCE_LINK_7*(c5*c6*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) + c6*s5*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3))) - (c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - DISTANCE_LINK_3*c1*c2*s3 - DISTANCE_LINK_3*c1*c3*s2;
    float J23 = - DISTANCE_LINK_7*(c6*s5*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) + c5*c6*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4))) - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) - (c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) - DISTANCE_LINK_3*c2*s1*s3 - DISTANCE_LINK_3*c3*s1*s2;
    float J33 = (c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) + DISTANCE_LINK_7*(c5*c6*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)) - c6*s5*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))) - (c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5) + DISTANCE_LINK_3*c2*c3 - DISTANCE_LINK_3*s2*s3;
    float J43 = s1;
    float J53 = -c1;
    uint  J63 = 0;

    float J14 = - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - DISTANCE_LINK_7*(c5*c6*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) + c6*s5*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3))) - (c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5);
    float J24 = - DISTANCE_LINK_6*(c6*s5*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) + c5*c6*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4))) - (DISTANCE_LINK_4 - DISTANCE_LINK_5*s5)*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)) - DISTANCE_LINK_5*c5*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3));
    float J34 = (c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_4 + DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5) + DISTANCE_LINK_7*(c5*c6*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)) - c6*s5*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))) - (c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5);
    float J44 = s1;
    float J54 = -c1;
    uint  J64 = 0;

    float J15 = - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - (DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5)*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) - DISTANCE_LINK_7*(c5*c6*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) + c6*s5*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)));
    float J25 = - DISTANCE_LINK_7*(c6*s5*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) + c5*c6*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4))) - (DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5)*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) - (DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5)*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4));
    float J35 = DISTANCE_LINK_7*(c5*c6*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)) - c6*s5*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))) - (c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4))*(DISTANCE_LINK_5*c5 + DISTANCE_LINK_6*s5) + (c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3))*(DISTANCE_LINK_6*c5 - DISTANCE_LINK_5*s5);
    float J45 = s1;
    float J55 = -c1;
    uint  J65 = 0;

    float J16 = -DISTANCE_LINK_7*(c6*s1 + c5*s6*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - s5*s6*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)));
    float J26 = DISTANCE_LINK_7*(c1*c6 - c5*s6*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3)) + s5*s6*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)));
    float J36 = -DISTANCE_LINK_7*(c5*s6*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4)) + s5*s6*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)));
    float J46 = - c5*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4)) - s5*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3));
    float J56 = - c5*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)) - s5*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3));
    float J66 = c5*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)) - s5*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4));
 
    jacobian[0][0] = J11;
    jacobian[0][1] = J12;
    jacobian[0][2] = J13;
    jacobian[0][3] = J14;
    jacobian[0][4] = J15;
    jacobian[0][5] = J16;

    jacobian[1][0] = J21;
    jacobian[1][1] = J22;
    jacobian[1][2] = J23;
    jacobian[1][3] = J24;
    jacobian[1][4] = J25;
    jacobian[1][5] = J26;

    jacobian[2][0] = J31;
    jacobian[2][1] = J32;
    jacobian[2][2] = J33;
    jacobian[2][3] = J34;
    jacobian[2][4] = J35;
    jacobian[2][5] = J36;

    jacobian[3][0] = J41;
    jacobian[3][1] = J42;
    jacobian[3][2] = J43;
    jacobian[3][3] = J44;
    jacobian[3][4] = J45;
    jacobian[3][5] = J46;
    
    jacobian[4][0] = J51;
    jacobian[4][1] = J52;
    jacobian[4][2] = J53;
    jacobian[4][3] = J54;
    jacobian[4][4] = J55;
    jacobian[4][5] = J56;

    jacobian[5][0] = J61;
    jacobian[5][1] = J62;
    jacobian[5][2] = J63;
    jacobian[5][3] = J64;
    jacobian[5][4] = J65;
    jacobian[5][5] = J66;
    
    /*
    float angular_geometric_jacobian[3][6] = {{J41,J42,J43,J44,J45,J46},{J51,J52,J53,J54,J55,J56},{J61,J62,J63,J64,J65,J66}};

    float R11 = c5*c6*(c1*c2*(c3*c4 - s3*s4) - c1*s2*(c3*s4 + c4*s3)) - s1*s6 - c6*s5*(c1*c2*(c3*s4 + c4*s3) + c1*s2*(c3*c4 - s3*s4));
    float R21 = c1*s6 - c6*s5*(c2*s1*(c3*s4 + c4*s3) + s1*s2*(c3*c4 - s3*s4)) + c5*c6*(c2*s1*(c3*c4 - s3*s4) - s1*s2*(c3*s4 + c4*s3));
    float R31 = c5*c6*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4)) + c6*s5*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3));
 
    float R32 = - c5*s6*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4)) - s5*s6*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3));
    float R33 = c5*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3)) - s5*(c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4));
 
    float roll = atanf(R32/R33); 
    float pitch = -asinf(R31); 
    float yaw = atanf(R21/R11); 

    float angular_geometric_to_analytical_jacobian[3][3] = {{1,0,sin(pitch)},{0,cos(roll),-sin(roll)*cos(pitch)},{0,sin(roll),cos(roll)*cos(pitch)}};
    float angular_geometric_to_analytical_jacobian_inverse[3][3] = {0};

    inverse(3,angular_geometric_to_analytical_jacobian,angular_geometric_to_analytical_jacobian_inverse);

    float angular_analytical_jacobian[3][3] = {0};

    multiply_matrices(3,3,6,angular_geometric_to_analytical_jacobian_inverse,angular_geometric_jacobian,angular_analytical_jacobian);
    
    jacobian[3][0] = angular_analytical_jacobian[1][0];
    jacobian[3][1] = angular_analytical_jacobian[1][1];
    jacobian[3][2] = angular_analytical_jacobian[1][2];
    jacobian[3][3] = angular_analytical_jacobian[1][3];
    jacobian[3][4] = angular_analytical_jacobian[1][4];
    jacobian[3][5] = angular_analytical_jacobian[1][5];

    jacobian[4][0] = angular_analytical_jacobian[2][0];
    jacobian[4][1] = angular_analytical_jacobian[2][1];
    jacobian[4][2] = angular_analytical_jacobian[2][2];
    jacobian[4][3] = angular_analytical_jacobian[2][3];
    jacobian[4][4] = angular_analytical_jacobian[2][4];
    jacobian[4][5] = angular_analytical_jacobian[2][5];
    */
}

float map_function(float input, float input_start, float input_end, float output_start, float output_end){
    float slope = (output_end - output_start) / (input_end - input_start);
    float output = output_start + slope * (input - input_start);

    return output;
}

void set_zero_position(){
    
    theta[0] = 0;
    theta[1] = M_PI/2;
    theta[2] = -M_PI/2;
    theta[3] = -M_PI/2;
    theta[4] = M_PI/2;
    theta[5] = 0;
    /*
    target_position[MOTOR_1] = map_function(theta[0],0,0,0,0);
    target_position[MOTOR_2] = map_function(theta[1],0,0,0,0);
    target_position[MOTOR_3] = map_function(theta[2],0,0,0,0);
    target_position[MOTOR_4] = map_function(theta[3],0,0,0,0);
    target_position[MOTOR_5] = map_function(theta[4],0,0,0,0);
    target_position[MOTOR_6] = map_function(theta[5],0,0,0,0);
    */
}

void claw_move(){
    if(claw_position == true){
        pwm_set_freq_duty(slice_motors[MOTOR_7], chan_motors[MOTOR_7], PWM_FREQ, 1.8/20);
    }
    else{
        pwm_set_freq_duty(slice_motors[MOTOR_7], chan_motors[MOTOR_7], PWM_FREQ, 1.4/20);
    }
    pwm_set_enabled(slice_motors[MOTOR_7], true);
}

void robot_move(uint size, bool debug, float lambda, float position[size]){

    float position_final[5];
    float position_initial[5];
    float x_final[3];
    float x_initial[3];
    float initial_angle[6];
    float jacobian_matrix[6][6];
    float jacobian_inverse[6][6];
    float position_difference[3];
    float total_distance[3];
    float velocity[6][1];
    float delta_angle[6][1];

    float total_difference[6];

    float initial_rotation_matrix[3][3] = {0};
    float final_rotation_matrix[3][3] = {0};
    float rotation_transpose1[3][3]= {0};
    float rotation_transpose2[3][3]= {0};
    float r_error1[3][3] = {0};
    float r_error2[3][3] = {0};
    float r_error[3][3] = {0};
    float angular_difference[3] = {0};

    float time_step = 0.001;
    float error = 0.05;
    float speed = 2;

    int32_t count = 0;
    uint64_t start;
    uint64_t end;
    uint64_t total_time = 0;
    uint64_t average_time = 0;

    for(int i=0; i<6; ++i){
        initial_angle[i] = theta[i];        
    }

    jacobian_function(theta[0], theta[1], theta[2], theta[3], theta[4], theta[5], jacobian_matrix);
    forward_kinematics(theta[0], theta[1], theta[2], theta[3], theta[4], theta[5], position_initial, initial_rotation_matrix);
    euler_to_rotation_matrix(position[3],position[4],final_rotation_matrix);
    for(int i=0; i<3; ++i){
        x_final[i] = position[i];
        x_initial[i] = position_initial[i];
        total_distance[i] = x_final[i] - x_initial[i];
        position_difference[i] = 1;
    }

    transpose(3,3,final_rotation_matrix,rotation_transpose2);

    while(norm(3, position_difference) >= error){
        start = time_us_64();
        //position_final = position_initial - position_difference
        add_subtract(3,x_final, x_initial, position_difference, false);

        transpose(3,3,initial_rotation_matrix,rotation_transpose1);
        multiply_matrices(3,3,3,final_rotation_matrix,rotation_transpose1,r_error1);
        multiply_matrices(3,3,3,rotation_transpose2,initial_rotation_matrix,r_error2);

        add_subtract_matrix(3, 3,r_error1,r_error2,r_error,false);

       // angular_difference[0] = (0.5)*(r_error[2][1] - r_error[1][2]);
       // angular_difference[1] = (0.5)*(r_error[0][2] - r_error[2][0]);
       // angular_difference[2] = (0.5)*(r_error[1][0] - r_error[0][1]);

        angular_difference[0] = 2*r_error[2][1];
        angular_difference[1] = 2*r_error[0][2];
        angular_difference[2] = 2*r_error[1][0];

        total_difference[0] = position_difference[0];
        total_difference[1] = position_difference[1];
        total_difference[2] = position_difference[2];
        total_difference[3] = angular_difference[0];
        total_difference[4] = angular_difference[1];
        total_difference[5] = angular_difference[2];

        //velocity = speed*(position_difference/norm(position_difference))
        calculate_velocity(6, total_difference, velocity, speed);

        pseudo_inverse(6,6,lambda,jacobian_matrix, jacobian_inverse);

        //Calculate delta_angle from delta_angle = J^{-1} * Velocity
        multiply_matrices(6,6,1,jacobian_inverse, velocity, delta_angle);

        //Calculate all new angles to send to motors
        for(int i=0; i<6; ++i){
            initial_angle[i] = delta_angle[i][0]*time_step + initial_angle[i];
            theta[i] = initial_angle[i];
        }

        //Calculate new position and jacobian
        forward_kinematics(initial_angle[0], initial_angle[1], initial_angle[2], initial_angle[3], initial_angle[4], initial_angle[5], position_initial, initial_rotation_matrix);

        jacobian_function(initial_angle[0], initial_angle[1], initial_angle[2], initial_angle[3], initial_angle[4], initial_angle[5], jacobian_matrix);

        for(int i=0; i<3; ++i){
            x_initial[i] = position_initial[i];
        }

        //Map angles to a value between 0 and 4095
        target_position[MOTOR_1] = map_function(theta[0],0,0,0,0);
        target_position[MOTOR_2] = map_function(theta[1],0,0,0,0);
        target_position[MOTOR_3] = map_function(theta[2],0,0,0,0);
        target_position[MOTOR_4] = map_function(theta[3],0,0,0,0);
        target_position[MOTOR_5] = map_function(theta[4],0,0,0,0);
        target_position[MOTOR_6] = map_function(theta[5],0,0,0,0);

        if(debug == true){
            printf("X: %0.3f \n", position_initial[0]);
            printf("Y: %0.3f \n", position_initial[1]);
            printf("Z: %0.3f \n", position_initial[2]);
            printf("PITCH: %0.3f \n", position_initial[3]);
            printf("YAW: %0.3f \n\n", position_initial[4]);
        }

        end = time_us_64();
        //printf("%llu \n",end-start);

        ++count;
       // if(count > 11000){error=4000;}
        if(count - 5000>= norm(3, total_distance)/(speed*time_step)){error = 4000;}
    } 
    //printf("X: %0.3f \n", position_initial[0]);
    //printf("Y: %0.3f \n", position_initial[1]);
    //printf("Z: %0.3f \n\n", position_initial[2]);
    //printf("count: %lu \n", count);
    //printf("average_time: %llu \n", average_time);
}

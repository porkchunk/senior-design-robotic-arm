#include "robot_commands.h"
#include "motor.h"
#include "pico/stdlib.h"
#include "main.h"
#include "pwm.h"
#include "robot_modes.h"
#include "pico/multicore.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float xyzpitch[6];
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

void forward_kinematics(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6, float xyzpitch[5]){
    xyzpitch[0] = (DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5))*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - DISTANCE_LINK_6*(sin(theta1)*sin(theta6) - cos(theta5)*cos(theta6)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) + cos(theta6)*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)))) + cos(theta1)*cos(theta2)*(DISTANCE_LINK_2 + DISTANCE_LINK_3*cos(theta3)) - DISTANCE_LINK_5*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - DISTANCE_LINK_3*cos(theta1)*sin(theta2)*sin(theta3);
    xyzpitch[1] = DISTANCE_LINK_6*(cos(theta1)*sin(theta6) - cos(theta6)*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) + cos(theta5)*cos(theta6)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)))) + (DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5))*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) + cos(theta2)*sin(theta1)*(DISTANCE_LINK_2 + DISTANCE_LINK_3*cos(theta3)) - DISTANCE_LINK_5*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - DISTANCE_LINK_3*sin(theta1)*sin(theta2)*sin(theta3);
    xyzpitch[2] = DISTANCE_LINK_1 + sin(theta2)*(DISTANCE_LINK_2 + DISTANCE_LINK_3*cos(theta3)) + (cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)))*(DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5)) + DISTANCE_LINK_3*cos(theta2)*sin(theta3) + DISTANCE_LINK_5*sin(theta5)*(cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)));
    
    float R11 = cos(theta5)*cos(theta6)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - sin(theta1)*sin(theta6) - cos(theta6)*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)));
    float R21 = cos(theta1)*sin(theta6) - cos(theta6)*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) + cos(theta5)*cos(theta6)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)));
    float R31 = cos(theta5)*cos(theta6)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) + cos(theta6)*sin(theta5)*(cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)));
    //float R32 = - cos(theta5)*sin(theta6)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - sin(theta5)*sin(theta6)*(cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)));
    //float R33 = cos(theta5)*(cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - sin(theta5)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)));
 
    //xyzpitch[3] = atanf(R32/R33); ROLL (NEED TO GET RID OF WE CANT ROLL OUR ROBOT)
    xyzpitch[3] = atanf(-R31/(sqrtf(powf(R11,2) + powf(R21,2)))); //PITCH
    xyzpitch[4] = atanf(R21/R11); //YAW

    
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
    
    xyzpitch[0] = T07[0][3]; //X
    xyzpitch[1] = T07[1][3]; //Y
    xyzpitch[2] = T07[2][3]; //Z
    xyzpitch[3] = atanf(T07[2][1]/T07[2][2]); //Roll
    xyzpitch[4] = atanf(-T07[2][0]/(sqrtf(powf(T07[0][0],2) + powf(T07[1][0],2)))); //Pitch
    xyzpitch[5] = atanf(T07[1][0]/T07[0][0]); //Yaw
    */

}

void jacobian_function(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6, float jacobian[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]){
    float J11 = DISTANCE_LINK_5*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - (DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5))*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - cos(theta2)*sin(theta1)*(DISTANCE_LINK_2 + DISTANCE_LINK_3*cos(theta3)) - DISTANCE_LINK_6*(cos(theta1)*sin(theta6) - cos(theta6)*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) + cos(theta5)*cos(theta6)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)))) + DISTANCE_LINK_3*sin(theta1)*sin(theta2)*sin(theta3);
    float J21 = (DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5))*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - DISTANCE_LINK_6*(sin(theta1)*sin(theta6) - cos(theta5)*cos(theta6)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) + cos(theta6)*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)))) + cos(theta1)*cos(theta2)*(DISTANCE_LINK_2 + DISTANCE_LINK_3*cos(theta3)) - DISTANCE_LINK_5*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - DISTANCE_LINK_3*cos(theta1)*sin(theta2)*sin(theta3);
    uint J31 = 0;
    //uint J41 = 0;
    uint J41 = 0;
    uint J51 = 1;

    float J12 = -(DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5))*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - DISTANCE_LINK_6*(cos(theta5)*cos(theta6)*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) + cos(theta6)*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)))) - cos(theta1)*sin(theta2)*(DISTANCE_LINK_2 + DISTANCE_LINK_3*cos(theta3)) - DISTANCE_LINK_5*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - DISTANCE_LINK_3*cos(theta1)*cos(theta2)*sin(theta3);
    float J22 = -DISTANCE_LINK_6*(cos(theta6)*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) + cos(theta5)*cos(theta6)*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)))) - (DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5))*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - sin(theta1)*sin(theta2)*(DISTANCE_LINK_2 + DISTANCE_LINK_3*cos(theta3)) - DISTANCE_LINK_5*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - DISTANCE_LINK_3*cos(theta2)*sin(theta1)*sin(theta3);
    float J32 = cos(theta2)*(DISTANCE_LINK_2 + DISTANCE_LINK_3*cos(theta3)) + DISTANCE_LINK_6*(cos(theta5)*cos(theta6)*(cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - cos(theta6)*sin(theta5)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)))) + (cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)))*(DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5)) - DISTANCE_LINK_3*sin(theta2)*sin(theta3) - DISTANCE_LINK_5*sin(theta5)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)));
    //float J42 = sin(theta1);    
    float J42 = -cos(theta1);
    uint  J52 = 0;


    float J13 = -(DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5))*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - DISTANCE_LINK_6*(cos(theta5)*cos(theta6)*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) + cos(theta6)*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)))) - DISTANCE_LINK_5*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - DISTANCE_LINK_3*cos(theta1)*cos(theta2)*sin(theta3) - DISTANCE_LINK_3*cos(theta1)*cos(theta3)*sin(theta2);
    float J23 = -DISTANCE_LINK_6*(cos(theta6)*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) + cos(theta5)*cos(theta6)*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)))) - (DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5))*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - DISTANCE_LINK_5*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - DISTANCE_LINK_3*cos(theta2)*sin(theta1)*sin(theta3) - DISTANCE_LINK_3*cos(theta3)*sin(theta1)*sin(theta2);
    float J33 = DISTANCE_LINK_6*(cos(theta5)*cos(theta6)*(cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - cos(theta6)*sin(theta5)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)))) + (cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)))*(DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5)) + DISTANCE_LINK_3*cos(theta2)*cos(theta3) - DISTANCE_LINK_3*sin(theta2)*sin(theta3) - DISTANCE_LINK_5*sin(theta5)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)));
    //float J43 = sin(theta1);
    float J43 = -cos(theta1);
    uint  J53 = 0;

    float J14 = -(DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5))*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - DISTANCE_LINK_6*(cos(theta5)*cos(theta6)*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) + cos(theta6)*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)))) - DISTANCE_LINK_5*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)));
    float J24 = -DISTANCE_LINK_6*(cos(theta6)*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) + cos(theta5)*cos(theta6)*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)))) - (DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5))*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - DISTANCE_LINK_5*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)));
    float J34 = DISTANCE_LINK_6*(cos(theta5)*cos(theta6)*(cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - cos(theta6)*sin(theta5)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)))) + (cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)))*(DISTANCE_LINK_4 + DISTANCE_LINK_5*cos(theta5)) - DISTANCE_LINK_5*sin(theta5)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)));
    //float J44 = sin(theta1);
    float J44 = -cos(theta1);
    uint  J54 = 0;

    float J15 = -DISTANCE_LINK_6*(cos(theta5)*cos(theta6)*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) + cos(theta6)*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)))) - DISTANCE_LINK_5*cos(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - DISTANCE_LINK_5*sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)));
    float J25 = -DISTANCE_LINK_6*(cos(theta6)*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) + cos(theta5)*cos(theta6)*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)))) - DISTANCE_LINK_5*cos(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - DISTANCE_LINK_5*sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)));
    float J35 = DISTANCE_LINK_6*(cos(theta5)*cos(theta6)*(cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - cos(theta6)*sin(theta5)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)))) + DISTANCE_LINK_5*cos(theta5)*(cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - DISTANCE_LINK_5*sin(theta5)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)));
    //float J45 = sin(theta1);
    float J45 = -cos(theta1);
    uint  J55 = 0;

    float J16 = -DISTANCE_LINK_6*(cos(theta6)*sin(theta1) + cos(theta5)*sin(theta6)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - sin(theta5)*sin(theta6)*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))));
    float J26 = DISTANCE_LINK_6*(cos(theta1)*cos(theta6) - cos(theta5)*sin(theta6)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) + sin(theta5)*sin(theta6)*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))));
    float J36 = -DISTANCE_LINK_6*(cos(theta5)*sin(theta6)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) + sin(theta5)*sin(theta6)*(cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))));
    //float J46 = -cos(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - sin(theta5)*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)));
    float J46 = -cos(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) - sin(theta5)*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)));
    float J56 = cos(theta5)*(cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) - sin(theta5)*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)));
 
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

}

float map_function(float input, float input_start, float input_end, float output_start, float output_end){
    float slope = (output_end - output_start) / (input_end - input_start);
    float output = output_start + slope * (input - input_start);

    return output;
}

void claw_move(){
    if(claw_position == true){
        pwm_set_freq_duty(slice_motors[MOTOR_6], chan_motors[MOTOR_6], PWM_FREQ, 2.15/20);
    }
    else{
        pwm_set_freq_duty(slice_motors[MOTOR_6], chan_motors[MOTOR_6], PWM_FREQ, 1.4/20);
    }
    pwm_set_enabled(slice_motors[MOTOR_6], true);
}

void set_zero_position(){
    duty_cycle_set(0, M_PI/2, -M_PI/2, 0, 1);
    motor_move(slice_motors, chan_motors);
    claw_move(slice_motors, chan_motors);

    sleep_ms(1000);
}

void set_initial_position(){
    xyzpitch[0] = STARTING_X;
    xyzpitch[1] = STARTING_Y;
    xyzpitch[2] = STARTING_Z;
    xyzpitch[3] = STARTING_PITCH;   
    xyzpitch[4] = STARTING_YAW;

    //robot_move(xyzpitch);
}


void robot_move(float xyzpitch[5]){
    //Convert current duty_cycle to angle based on model of robot
    //float theta1 = map_function(20*duty_cycle_motors.motor1, 0.5, 2.5, -M_PI/2, M_PI/2);
    //float theta2 = map_function(20*duty_cycle_motors.motor2, 0.5, 2.5, M_PI, 0);
    //float theta3 = map_function(20*duty_cycle_motors.motor3, 0.56, 2.06, (-3*M_PI)/4, 0);
   // float theta4 = map_function(20*duty_cycle_motors.motor4, 0.5, 2.5, -M_PI/2, M_PI/2);

    float theta1 = 0;
    float theta2 = M_PI/2;
    float theta3 = -M_PI/2;
    float theta4 = 0;
    float theta5 = 0;
    float theta6 = 0;

    float position_final[5];
    float position_initial[5];
    float initial_angle[6];
    float jacobian_matrix[5][6];
    float jacobian_inverse[6][5];
    float position_difference[5] = {1, 1, 1, 1, 1};
    float total_distance[5];
    float velocity[5][1];
    float delta_angle[6][1];
    float time_step = 0.01;
    float error = 0.03;
    float speed = 1;

    uint32_t count = 0;

    jacobian_function(theta1, theta2, theta3, theta4, theta5, theta6, jacobian_matrix);
    forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6, position_initial);

    for(int i=0; i<5; ++i){
        position_final[i] = xyzpitch[i];
    }

    for(int i=0; i<5; ++i){
        total_distance[i] = position_final[i] - position_initial[i];
    }

    initial_angle[0] = theta1;
    initial_angle[1] = theta2;
    initial_angle[2] = theta3;
    initial_angle[3] = theta4;
    initial_angle[4] = theta5;
    initial_angle[5] = theta6;

    while(norm(position_difference) >= error){
        //position_final = position_initial - position_difference
        add_subtract_matrix(position_final, position_initial, position_difference, false);

        //velocity = speed*(position_difference/norm(position_difference))
        calculate_velocity(position_difference, velocity, speed);

        pseudo_inverse(jacobian_matrix, jacobian_inverse);

        //Calculate delta_angle from delta_angle = J^{-1} * Velocity
        multiply_matrices(6,5,1,jacobian_inverse, velocity, delta_angle);

        //Calculate all new angles to send to motors
        initial_angle[0] = delta_angle[0][0]*time_step + initial_angle[0];
        initial_angle[1] = delta_angle[1][0]*time_step + initial_angle[1];
        initial_angle[2] = delta_angle[2][0]*time_step + initial_angle[2];
        initial_angle[3] = delta_angle[3][0]*time_step + initial_angle[3];
        initial_angle[4] = delta_angle[4][0]*time_step + initial_angle[4];
        initial_angle[5] = delta_angle[5][0]*time_step + initial_angle[5];

        //Send new angles to motors
       // duty_cycle_set(initial_angle[0], initial_angle[1], initial_angle[2], initial_angle[3], 1);
       // motor_move(slice_motors, chan_motors);

        //Calculate new position and jacobian
        forward_kinematics(initial_angle[0], initial_angle[1], initial_angle[2], initial_angle[3], initial_angle[4], initial_angle[5], position_initial);
        jacobian_function(initial_angle[0], initial_angle[1], initial_angle[2], initial_angle[3], initial_angle[4], initial_angle[5], jacobian_matrix);

        //End robot_move if count goes too high
        ++count;
        if(count >= norm(total_distance)/(speed*time_step)){error = 4000;}
    }
}

#ifndef ROBOT_COMMANDS_H
#define ROBOT_COMMANDS_H

#include "matrix.h"
#include "pico/stdlib.h"

#define DISTANCE_LINK_1 6
#define DISTANCE_LINK_2 8 
#define DISTANCE_LINK_3 13
#define DISTANCE_LINK_4 14
#define DISTANCE_LINK_5 12
#define DISTANCE_LINK_6 14

#define POSITION_SIZE 3

extern float position[POSITION_SIZE];
extern volatile bool claw_position;
extern float theta[6];

void transform_matrix(float alpha, float a, float d, float theta, float T[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
void forward_kinematics(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6, float xyzpitch[POSITION_SIZE]);
void jacobian_function(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6, float jacobian[3][6]);
float map_function(float input, float input_start, float input_end, float output_start, float output_end);
void claw_move();
void robot_move(int size, float xyzpitch[size]);
void set_initial_position();
void set_zero_position();

#endif
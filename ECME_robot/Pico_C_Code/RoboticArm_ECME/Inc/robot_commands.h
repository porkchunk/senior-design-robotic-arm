#ifndef ROBOT_COMMANDS_H
#define ROBOT_COMMANDS_H

#include "matrix.h"
#include "pico/stdlib.h"

#define DISTANCE_LINK_1 1.5
#define DISTANCE_LINK_2 7.2
#define DISTANCE_LINK_3 3
#define DISTANCE_LINK_4 7
#define DISTANCE_LINK_5 0.95
#define DISTANCE_LINK_6 2.5
#define DISTANCE_LINK_7 3.6

#define POSITION_SIZE 5

extern float position[POSITION_SIZE];
extern volatile bool claw_position;
extern float theta[6];

void transform_matrix(float alpha, float a, float d, float theta, float T[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
void forward_kinematics(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6, float position[POSITION_SIZE], float rotation_matrix[3][3]);
void jacobian_function(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6, float jacobian[6][6]);
float map_function(float input, float input_start, float input_end, float output_start, float output_end);
void robot_move(uint size, bool debug, float lambda, float position[size]);
void set_zero_position();
void claw_move();

#endif
#ifndef ROBOT_COMMANDS_H
#define ROBOT_COMMANDS_H

#include "matrix.h"
#include "pico/stdlib.h"

#define TRANSFORM_ROW_SIZE 1
#define TRANSFORM_COLUMN_SIZE 4
#define DISTANCE_LINK_1 9.5
#define DISTANCE_LINK_2 10.4 
#define DISTANCE_LINK_3 8.75
#define DISTANCE_LINK_4 16.25

extern float xyzpitch[4];
extern volatile bool claw_position;

void transform_matrix(float alpha, float a, float d, float theta, float T[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
void forward_kinematics(float theta1, float theta2, float theta3, float theta4, float xyzpitch[4]);
void jacobian_function(float theta1, float theta2, float theta3, float theta4, float jacobian[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
float map_function(float input, float input_start, float input_end, float output_start, float output_end);
void claw_move();
void robot_move(float xyzpitch[4]);
void set_initial_position();
void set_zero_position();

#endif
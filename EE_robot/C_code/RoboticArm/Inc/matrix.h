#ifndef MATRIX_H
#define MATRIX_H

#include "pico/stdlib.h"

#define MATRIX_ROW_SIZE 4
#define MATRIX_COL_SIZE 4
#define MATRIX_SIZE 4

void multiply_matrices(float first[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float second[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float result[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
void multiply_matrices_angle(float first[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float second[MATRIX_ROW_SIZE][1],float result[MATRIX_ROW_SIZE][1]);
int inverse(int matrix_size, float mat[matrix_size][matrix_size], float inverse[matrix_size][matrix_size]);
void add_subtract_matrix(float matrix1[4],float matrix2[4],float result_matrix[4], bool add);
float norm(float position_difference[4]);
void calculate_velocity(float position_difference[4],float velocity[4][1],float speed);

#endif
#ifndef MATRIX_H
#define MATRIX_H

#include "pico/stdlib.h"

#define MATRIX_ROW_SIZE 5
#define MATRIX_COL_SIZE 6
#define MATRIX_SIZE 6

void multiply_matrices(int m, int n, int p, float first[][n], float second[][p], float result[][p]);
void display(int m, int n, float result[m][n]);
void add_subtract_matrix(float matrix1[5],float matrix2[5],float result_matrix[5], bool add);
float norm(float position_difference[5]);
void calculate_velocity(float position_difference[5],float velocity[5][1],float speed);
void transpose(float matrix[5][6], float transposed_matrix[6][5]);
void display_transpose_and_pseudo_inverse(float result[MATRIX_COL_SIZE][MATRIX_ROW_SIZE]);
void pseudo_inverse(float matrix[MATRIX_ROW_SIZE][MATRIX_COL_SIZE], float pseudo_inverse[MATRIX_COL_SIZE][MATRIX_ROW_SIZE]);
int inverse(int matrix_size, float mat[matrix_size][matrix_size], float inverse[matrix_size][matrix_size]);

#endif
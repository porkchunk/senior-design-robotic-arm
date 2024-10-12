#ifndef MATRIX_H
#define MATRIX_H

#include "pico/stdlib.h"

#define MATRIX_ROW_SIZE 5
#define MATRIX_COL_SIZE 6
#define MATRIX_SIZE 6

void multiply_matrices(int m, int n, int p, float first[][n], float second[][p], float result[][p]);
void display(int m, int n, float result[m][n]);
void add_subtract_matrix(int size, float matrix1[size],float matrix2[size],float result_matrix[size], bool add);
float norm(int size, float position_difference[size]);
void calculate_velocity(int size, float position_difference[size],float velocity[size][1],float speed);
void transpose(int m, int n, float matrix[m][n], float transposed_matrix[n][m]);
void pseudo_inverse(int m, int n, float matrix[m][n], float pseudo_inverse[n][m]);
int inverse(int matrix_size, float mat[matrix_size][matrix_size], float inverse[matrix_size][matrix_size]);

#endif
#ifndef MATRIX_H
#define MATRIX_H

#include "pico/stdlib.h"

#define MATRIX_ROW_SIZE 5
#define MATRIX_COL_SIZE 6
#define MATRIX_SIZE 6

void multiply_matrices(int m, int n, int p, float first[][n], float second[][p], float result[][p]);
void display(int m, int n, float result[m][n]);
void add_subtract_matrix(int m, int n, float matrix1[m][n],float matrix2[m][n],float result_matrix[m][n], bool add);
float norm(int size, float position_difference[size]);
void calculate_velocity(int size, float position_difference[size],float velocity[size][1],float speed);
void transpose(int m, int n, float matrix[m][n], float transposed_matrix[n][m]);
void pseudo_inverse(int m, int n, float matrix[m][n], float pseudo_inverse[n][m]);
int inverse(int matrix_size, float mat[matrix_size][matrix_size], float inverse[matrix_size][matrix_size]);
void euler_to_rotation_matrix(float pitch, float yaw, float rotation_matrix[3][3]);
void add_subtract(int m, float matrix1[m],float matrix2[m],float result_matrix[m], bool add);

#endif
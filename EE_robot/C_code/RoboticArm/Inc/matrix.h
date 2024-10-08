#ifndef MATRIX_H
#define MATRIX_H

#include "pico/stdlib.h"

#define MATRIX_ROW_SIZE 4
#define MATRIX_COL_SIZE 4
#define N 4

void multiply_matrices(float first[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float second[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float result[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
void multiply_matrices_angle(float first[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float second[MATRIX_ROW_SIZE][1],float result[MATRIX_ROW_SIZE][1]);
void display(float result[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
void getCofactor(float mat[N][N], float temp[N][N], int p, int q, int n);
float determinant(float mat[N][N], int n);
void adjoint(float mat[N][N], float adj[N][N]);
int inverse(float mat[N][N], float inverse[N][N]);
void add_subtract_matrix(float matrix1[4],float matrix2[4],float result_matrix[4], bool add);
float norm(float position_difference[4]);
void calculate_velocity(float position_difference[4],float velocity[4][1],float speed);

#endif
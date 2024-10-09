#include "matrix.h"
#include <stdio.h>
#include <math.h>

// Function to get cofactor of mat[p][q] in temp[][]. n is the current dimension of mat[][]
void getCofactor(float mat[MATRIX_SIZE][MATRIX_SIZE], float temp[MATRIX_SIZE][MATRIX_SIZE], int p, int q, int n) {
    int i = 0, j = 0;

    // Looping for each element of the matrix
    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {
            // Copying into temporary matrix only those elements which are not in the given row and column
            if (row != p && col != q) {
                temp[i][j++] = mat[row][col];

                // Row is filled, so increase row index and reset column index
                if (j == n - 1) {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

// Recursive function to find determinant of matrix mat[][]
float determinant(float mat[MATRIX_SIZE][MATRIX_SIZE], int n) {
    float D = 0;  // Initialize result

    // Base case: if matrix contains single element
    if (n == 1)
        return mat[0][0];

    float temp[MATRIX_SIZE][MATRIX_SIZE];  // To store cofactors
    int sign = 1;  // To store sign multiplier

    // Iterate for each element of first row
    for (int f = 0; f < n; f++) {
        // Getting Cofactor of mat[0][f]
        getCofactor(mat, temp, 0, f, n);
        D += sign * mat[0][f] * determinant(temp, n - 1);

        // Terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}

// Function to get adjoint of mat[MATRIX_SIZE][MATRIX_SIZE] in adj[MATRIX_SIZE][MATRIX_SIZE]
void adjoint(float mat[MATRIX_SIZE][MATRIX_SIZE], float adj[MATRIX_SIZE][MATRIX_SIZE]) {
    if (MATRIX_SIZE == 1) {
        adj[0][0] = 1;
        return;
    }

    // temp is used to store cofactors
    int sign = 1;
    float temp[MATRIX_SIZE][MATRIX_SIZE];

    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            // Get cofactor of mat[i][j]
            getCofactor(mat, temp, i, j, MATRIX_SIZE);

            // Sign of adj[j][i] positive if sum of row and column indexes is even.
            sign = ((i + j) % 2 == 0) ? 1 : -1;

            // Interchanging rows and columns to get the transpose of the cofactor matrix
            adj[j][i] = (sign) * (determinant(temp, MATRIX_SIZE - 1));
        }
    }
}

// Function to calculate the inverse of a matrix. Returns 1 if the matrix is invertible, otherwise 0.
int inverse(float mat[MATRIX_SIZE][MATRIX_SIZE], float inverse[MATRIX_SIZE][MATRIX_SIZE]) {
    // Find determinant of matrix
    float det = determinant(mat, MATRIX_SIZE);
    if (det == 0) {
        printf("Matrix is singular, cannot find its inverse\n");
        return 0;
    }

    // Find adjoint
    float adj[MATRIX_SIZE][MATRIX_SIZE];
    adjoint(mat, adj);

    // Inverse is adjoint divided by determinant
    for (int i = 0; i < MATRIX_SIZE; i++)
        for (int j = 0; j < MATRIX_SIZE; j++)
            inverse[i][j] = adj[i][j] / det;

    return 1;
}

// function to multiply two matrices 4x4 and 4x4
void multiply_matrices(float first[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float second[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float result[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]) {

   // Initializing elements of matrix mult to 0.
   for (int i = 0; i < MATRIX_ROW_SIZE; ++i) {
      for (int j = 0; j < MATRIX_COL_SIZE; ++j) {
         result[i][j] = 0;
      }
   }

   // Multiplying first and second matrices and storing it in result
   for (int i = 0; i < MATRIX_ROW_SIZE; ++i) {
      for (int j = 0; j < MATRIX_COL_SIZE; ++j) {
         for (int k = 0; k < MATRIX_COL_SIZE; ++k) {
            result[i][j] += first[i][k] * second[k][j];
         }
      }
   }
}

// function to multiply 4x4 and 4x1 matrix
void multiply_matrices_angle(float first[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float second[MATRIX_ROW_SIZE][1],float result[MATRIX_ROW_SIZE][1]) {

   // Initializing elements of matrix mult to 0.
   for (int i = 0; i < MATRIX_ROW_SIZE; ++i) {
      for (int j = 0; j < 1; ++j) {
         result[i][j] = 0;
      }
   }

   // Multiplying first and second matrices and storing it in result
   for (int i = 0; i < MATRIX_ROW_SIZE; ++i) {
      for (int j = 0; j < 1; ++j) {
         for (int k = 0; k < MATRIX_COL_SIZE; ++k) {
            result[i][j] += first[i][k] * second[k][j];
         }
      }
   }
}

void add_subtract_matrix(float matrix1[4],float matrix2[4],float result_matrix[4], bool add){
    if(add == true){
        for(int i=0; i<4; ++i){
            result_matrix[i] = matrix1[i] + matrix2[i];
        }
    }
    else{
        for(int i=0; i<4; ++i){
            result_matrix[i] = matrix1[i] - matrix2[i];
        }
    }
}

float norm(float position_difference[4]){
    float norm = 0;
    norm = sqrtf(powf(position_difference[0], 2) + powf(position_difference[1], 2) + powf(position_difference[2], 2) + powf(position_difference[3], 2));
    return norm;
}

void calculate_velocity(float position_difference[4],float velocity[4][1],float speed){
    for(int i=0; i<4; ++i){
        velocity[i][0] = speed*(position_difference[i]/norm(position_difference));
    }
}

// function to display the matrix
void display(float result[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]) {

   printf("\nOutput Matrix:\n");
   for (int i = 0; i < MATRIX_ROW_SIZE; ++i) {
      for (int j = 0; j < MATRIX_COL_SIZE; ++j) {
         printf("%.2f  ", result[i][j]);
         if (j == MATRIX_COL_SIZE - 1)
            printf("\n");
      }
   }
}
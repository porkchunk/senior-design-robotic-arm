#include "matrix.h"
#include <stdio.h>
#include <math.h>

int inverse(int matrix_size, float mat[matrix_size][matrix_size], float inverse[matrix_size][matrix_size]) {
    // Augmenting the matrix with the identity matrix
    float aug[matrix_size][2 * matrix_size];
    for (int i = 0; i < matrix_size; i++) {
        for (int j = 0; j < matrix_size; j++) {
            aug[i][j] = mat[i][j];  // Copy the original matrix
            aug[i][j + matrix_size] = (i == j) ? 1.0 : 0.0;  // Augment with identity matrix
        }
    }

    // Performing Gaussian elimination
    for (int i = 0; i < matrix_size; i++) {
        // Make the diagonal element 1 by dividing the entire row by that element
        if (aug[i][i] == 0) {
            // If the pivot is zero, try to swap with a non-zero row below it
            bool swapped = false;
            for (int k = i + 1; k < matrix_size; k++) {
                if (aug[k][i] != 0) {
                    // Swap the rows
                    for (int j = 0; j < 2 * matrix_size; j++) {
                        float temp = aug[i][j];
                        aug[i][j] = aug[k][j];
                        aug[k][j] = temp;
                    }
                    swapped = true;
                    break;
                }
            }
            if (!swapped) {
                printf("Matrix is singular, cannot find its inverse.\n");
                return 0;
            }
        }

        // Normalize the current row
        float diagElement = aug[i][i];
        for (int j = 0; j < 2 * matrix_size; j++) {
            aug[i][j] /= diagElement;
        }

        // Make other elements in the current column 0 by subtracting the appropriate multiple of the current row
        for (int k = 0; k < matrix_size; k++) {
            if (k != i) {
                float factor = aug[k][i];
                for (int j = 0; j < 2 * matrix_size; j++) {
                    aug[k][j] -= factor * aug[i][j];
                }
            }
        }
    }

    // Extract the inverse matrix from the augmented matrix
    for (int i = 0; i < matrix_size; i++) {
        for (int j = 0; j < matrix_size; j++) {
            inverse[i][j] = aug[i][j + matrix_size];
        }
    }

    return 1;
}
/*
void pseudo_inverse(float lambda, float matrix[4][4], float pseudo_inverse[4][4]){
    int m = 4;
    int n = 4;
    float first_term_linearly_surjective[m][m];
    float first_term_linearly_injective[n][n];
    float first_term_linearly_surjective_inverse[m][m];
    float first_term_linearly_injective_inverse[n][n];
    float intermediate_term[m][m];
    float matrix_transpose[n][m];
    
    float identity[4][4] = {{lambda*1,0,0,0},{0,lambda*1,0,0},{0,0,lambda*1,0},{0,0,0,lambda*1}};

    transpose(4,4,matrix, matrix_transpose);
    multiply_matrices(matrix, matrix_transpose, intermediate_term);
    add_subtract_matrix(intermediate_term, identity, first_term_linearly_surjective, true);
    inverse(4, first_term_linearly_surjective, first_term_linearly_surjective_inverse);
    multiply_matrices(matrix_transpose, first_term_linearly_surjective_inverse, pseudo_inverse);
}
*/

void pseudo_inverse(float matrix[4][4], float pseudo_inverse[4][4]){
    uint64_t start;
    uint64_t end;
    float first_term_linearly_surjective[4][4];
    float first_term_linearly_injective[4][4];
    float first_term_linearly_surjective_inverse[4][4];
    float first_term_linearly_injective_inverse[4][4];
    float matrix_transpose[4][4];
    transpose(4, 4, matrix, matrix_transpose);
    multiply_matrices(matrix, matrix_transpose, first_term_linearly_surjective);

    if(inverse(4, first_term_linearly_surjective, first_term_linearly_surjective_inverse) == 1){
        multiply_matrices(matrix_transpose, first_term_linearly_surjective_inverse, pseudo_inverse);
    }
    else{
        multiply_matrices(matrix_transpose, matrix, first_term_linearly_injective);
        inverse(4, first_term_linearly_injective, first_term_linearly_injective_inverse);
        multiply_matrices(first_term_linearly_injective_inverse, matrix_transpose, pseudo_inverse);
    }
}

void transpose(int m, int n, float matrix[m][n], float transposed_matrix[n][m]){
    for(int i=0;i<n;++i){
        for(int j=0;j<m;++j){
            transposed_matrix[i][j] = matrix[j][i];
        }
    }
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
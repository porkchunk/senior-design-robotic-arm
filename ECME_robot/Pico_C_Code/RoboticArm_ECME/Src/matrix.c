#include "matrix.h"
#include "pico/stdlib.h"
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

void transpose(float matrix[5][6], float transposed_matrix[6][5]){
    for(int i=0;i<6;++i){
        for(int j=0;j<5;++j){
            transposed_matrix[i][j] = matrix[j][i];
        }
    }
}

void multiply_matrices(int m, int n, int p, float first[][n], float second[][p], float result[][p]) {
   // Initializing elements of result matrix to 0.
   for (int i = 0; i < m; ++i) {
      for (int j = 0; j < p; ++j) {
         result[i][j] = 0;
      }
   }

   // Multiplying first and second matrices and storing it in result
   for (int i = 0; i < m; ++i) {
      for (int j = 0; j < p; ++j) {
         for (int k = 0; k < n; ++k) {
            result[i][j] += first[i][k] * second[k][j];
         }
      }
   }
}

// function to multiply 6x5 and 5x1 matrix
void multiply_matrices_angle(float first[6][5],float second[5][1],float result[6][1]) {

   // Initializing elements of matrix mult to 0.
   for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 1; ++j) {
         result[i][j] = 0;
      }
   }

   // Multiplying first and second matrices and storing it in result
   for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 1; ++j) {
         for (int k = 0; k < 5; ++k) {
            result[i][j] += first[i][k] * second[k][j];
         }
      }
   }
}

void add_subtract_matrix(float matrix1[5],float matrix2[5],float result_matrix[5], bool add){
    if(add == true){
        for(int i=0; i<5; ++i){
            result_matrix[i] = matrix1[i] + matrix2[i];
        }
    }
    else{
        for(int i=0; i<5; ++i){
            result_matrix[i] = matrix1[i] - matrix2[i];
        }
    }
}

float norm(float position_difference[5]){
    float norm = 0;
    norm = sqrtf(powf(position_difference[0], 2) + powf(position_difference[1], 2) + powf(position_difference[2], 2) + powf(position_difference[3], 2) + powf(position_difference[4], 2));
    return norm;
}

void calculate_velocity(float position_difference[5],float velocity[5][1],float speed){
    for(int i=0; i<5; ++i){
        velocity[i][0] = speed*(position_difference[i]/norm(position_difference));
    }
}

// function to display the matrix
void display(int m, int n, float result[m][n]) {

   printf("\nOutput Matrix:\n");
   for (int i = 0; i < m; ++i) {
      for (int j = 0; j < n; ++j) {
         printf("%.2f  ", result[i][j]);
         if (j == n - 1)
            printf("\n");
      }
   }
}

//function to display transpose
void display_transpose_and_pseudo_inverse(float result[MATRIX_COL_SIZE][MATRIX_ROW_SIZE]) {

   printf("\nOutput Matrix:\n");
   for (int i = 0; i < MATRIX_COL_SIZE; ++i) {
      for (int j = 0; j < MATRIX_ROW_SIZE; ++j) {
         printf("%.2f  ", result[i][j]);
         if (j == MATRIX_ROW_SIZE - 1)
            printf("\n");
      }
   }
}

void pseudo_inverse(float matrix[MATRIX_ROW_SIZE][MATRIX_COL_SIZE], float pseudo_inverse[MATRIX_COL_SIZE][MATRIX_ROW_SIZE]){
    uint64_t start;
    uint64_t end;
    float first_term_linearly_surjective[5][5];
    float first_term_linearly_injective[6][6];
    float first_term_linearly_surjective_inverse[5][5];
    float first_term_linearly_injective_inverse[6][6];
    float matrix_transpose[MATRIX_COL_SIZE][MATRIX_ROW_SIZE];

    start = time_us_64();
    transpose(matrix, matrix_transpose);
    end = time_us_64();

    printf("Transpose: %llu \n", end - start);

    start = time_us_64();
    multiply_matrices(5, 6, 5, matrix, matrix_transpose, first_term_linearly_surjective);
    end = time_us_64();

    printf("Multiply: %llu \n", end - start);

    start = time_us_64();
    if(inverse(5, first_term_linearly_surjective, first_term_linearly_surjective_inverse) == 1){
        end = time_us_64();

        printf("Inverse: %llu \n", end - start);

        start = time_us_64();

        multiply_matrices(6, 5, 5, matrix_transpose, first_term_linearly_surjective_inverse, pseudo_inverse);

        end = time_us_64();

    printf("Multiply: %llu \n\n", end - start);
    }
    /*
    else{
        multiply_matrices(6, 5, 6, matrix_transpose, matrix, first_term_linearly_injective);
        inverse(first_term_linearly_injective, first_term_linearly_injective_inverse);
        multiply_matrices(5, 6, 5, first_term_linearly_injective_inverse, matrix_transpose, pseudo_inverse);
    }
    */
}
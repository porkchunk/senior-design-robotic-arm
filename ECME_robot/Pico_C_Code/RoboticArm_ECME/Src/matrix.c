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
                //printf("Matrix is singular, cannot find its inverse.\n");
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

void transpose(int m, int n, float matrix[m][n], float transposed_matrix[n][m]){
    for(int i=0;i<n;++i){
        for(int j=0;j<m;++j){
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

void add_subtract_matrix(int size, float matrix1[size],float matrix2[size],float result_matrix[size], bool add){
    if(add == true){
        for(int i=0; i<size; ++i){
            result_matrix[i] = matrix1[i] + matrix2[i];
        }
    }
    else{
        for(int i=0; i<size; ++i){
            result_matrix[i] = matrix1[i] - matrix2[i];
        }
    }
}

float norm(int size, float position_difference[size]){
    float norm_squared = 0;
    float norm;

    for(int i=0; i<size; ++i){
        norm_squared = norm_squared + powf(position_difference[i], 2);
    }    
    
    norm = sqrtf(norm_squared);
    return norm;
}

void calculate_velocity(int size, float position_difference[size],float velocity[size][1],float speed){
    for(int i=0; i<size; ++i){
        velocity[i][0] = speed*(position_difference[i]/norm(size, position_difference));
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

void pseudo_inverse(int m, int n, float matrix[m][n], float pseudo_inverse[n][m]){
    uint64_t start;
    uint64_t end;
    float first_term_linearly_surjective[m][m];
    float first_term_linearly_injective[n][n];
    float first_term_linearly_surjective_inverse[m][m];
    float first_term_linearly_injective_inverse[n][n];
    float matrix_transpose[n][m];
    transpose(m, n, matrix, matrix_transpose);
    multiply_matrices(m, n, m, matrix, matrix_transpose, first_term_linearly_surjective);

    if(inverse(m, first_term_linearly_surjective, first_term_linearly_surjective_inverse) == 1){
        multiply_matrices(n, m, m, matrix_transpose, first_term_linearly_surjective_inverse, pseudo_inverse);
    }
    else{
        multiply_matrices(n, m, n, matrix_transpose, matrix, first_term_linearly_injective);
        inverse(n, first_term_linearly_injective, first_term_linearly_injective_inverse);
        multiply_matrices(m, n, m, first_term_linearly_injective_inverse, matrix_transpose, pseudo_inverse);
    }
}
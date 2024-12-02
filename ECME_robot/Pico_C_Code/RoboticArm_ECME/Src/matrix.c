#include "matrix.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

int inverse(int matrix_size, float mat[matrix_size][matrix_size], float inverse[matrix_size][matrix_size]) {
    float aug[matrix_size][2 * matrix_size];
    for (int i = 0; i < matrix_size; i++) {
        for (int j = 0; j < matrix_size; j++) {
            aug[i][j] = mat[i][j];  
            aug[i][j + matrix_size] = (i == j) ? 1.0 : 0.0; 
        }
    }

    for (int i = 0; i < matrix_size; i++) {
        if (aug[i][i] == 0) {
            bool swapped = false;
            for (int k = i + 1; k < matrix_size; k++) {
                if (aug[k][i] != 0) {
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
                for (int i = 0; i < matrix_size; i++) {
                    for (int j = 0; j < matrix_size; j++) {
                          inverse[i][j] = 0;
                      }
                 }
                printf("Matrix is singular, cannot find its inverse.\n");
                return 0;
            }
        }

    
        float diagElement = aug[i][i];
        for (int j = 0; j < 2 * matrix_size; j++) {
            aug[i][j] /= diagElement;
        }


        for (int k = 0; k < matrix_size; k++) {
            if (k != i) {
                float factor = aug[k][i];
                for (int j = 0; j < 2 * matrix_size; j++) {
                    aug[k][j] -= factor * aug[i][j];
                }
            }
        }
    }


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

void add_subtract(int m, float matrix1[m],float matrix2[m],float result_matrix[m], bool add){
    if(add == true){
        for(int i=0; i<m; ++i){
                result_matrix[i] = matrix1[i] + matrix2[i];
            }
        }
    else{
        for(int i=0; i<m; ++i){
                result_matrix[i] = matrix1[i] - matrix2[i];
        }
    }
}

void add_subtract_matrix(int m, int n, float matrix1[m][n],float matrix2[m][n],float result_matrix[m][n], bool add){
    if(add == true){
        for(int i=0; i<m; ++i){
            for(int j=0; j<n; ++j){
                result_matrix[i][j] = matrix1[i][j] + matrix2[i][j];
            }
        }
    }
    else{
        for(int i=0; i<m; ++i){
            for(int j=0; j<n; ++j){
                result_matrix[i][j] = matrix1[i][j] - matrix2[i][j];
            }
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
        if(j < n){
            if(result[i][j+1] < 0){
                printf("%.2f ", result[i][j]);
            }
            else{
                printf("%.2f  ", result[i][j]);
            }
        }
        if (j == n - 1)
        printf("\n");
      }
   }
}

void pseudo_inverse(int m, int n, float lambda, float matrix[m][n], float pseudo_inverse[n][m]){
    float first_term_linearly_surjective[m][m];
    float first_term_linearly_injective[n][n];
    float first_term_linearly_surjective_inverse[m][m];
    float first_term_linearly_injective_inverse[n][n];
    float intermediate_term[m][m];
    float matrix_transpose[n][m];
    
    float identity[6][6] = {{lambda*1,0,0,0,0,0},{0,lambda*1,0,0,0,0},{0,0,lambda*1,0,0,0},{0,0,0,lambda*1,0,0},{0,0,0,0,lambda*1,0},{0,0,0,0,0,lambda*1}};

    transpose(m, n, matrix, matrix_transpose);
    multiply_matrices(m, m, n, matrix, matrix_transpose, intermediate_term);
    add_subtract_matrix(m, m, intermediate_term, identity, first_term_linearly_surjective, true);
    inverse(m, first_term_linearly_surjective, first_term_linearly_surjective_inverse);
    multiply_matrices(n, n, m, matrix_transpose, first_term_linearly_surjective_inverse, pseudo_inverse);

    /*
    transpose(m, n, matrix, matrix_transpose);
    multiply_matrices(m, n, m, matrix, matrix_transpose, first_term_linearly_surjective);

    if(inverse(m, first_term_linearly_surjective, first_term_linearly_surjective_inverse) == 1){
        multiply_matrices(n, m, m, matrix_transpose, first_term_linearly_surjective_inverse, pseudo_inverse);
    }
    else{
        printf("injective \n");
        multiply_matrices(n, m, n, matrix_transpose, matrix, first_term_linearly_injective);
        inverse(n, first_term_linearly_injective, first_term_linearly_injective_inverse);
        multiply_matrices(n, n, m, first_term_linearly_injective_inverse, matrix_transpose, pseudo_inverse);
    }
   */
}

void euler_to_rotation_matrix(float pitch, float yaw, float rotation_matrix[3][3]){    
    float cp = cos(pitch);
    float sp = sin(pitch);
    float cy = cos(yaw);
    float sy = sin(yaw);

    rotation_matrix[0][0] = cp*cy;
    rotation_matrix[1][0] = cp*sy;
    rotation_matrix[2][0] = -sp;

    rotation_matrix[0][1] = -sy;
    rotation_matrix[1][1] = cy;
    rotation_matrix[2][1] = 0;

    rotation_matrix[0][2] = cy*sp;
    rotation_matrix[1][2] = sp*sy;
    rotation_matrix[2][2] = cp;
}
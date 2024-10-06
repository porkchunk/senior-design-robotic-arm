#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#ifndef M_PI
#define M_PI 3.14159265358
#endif

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define TRANSFORM_ROW_SIZE 1
#define TRANSFORM_COLUMN_SIZE 4
#define MATRIX_ROW_SIZE 4
#define MATRIX_COL_SIZE 4
#define DISTANCE_LINK_1 9.5
#define DISTANCE_LINK_2 10.4 
#define DISTANCE_LINK_3 8.75
#define DISTANCE_LINK_4 16.25

#define NUMBER_OF_MOTORS 5
#define PIN_MOTOR_1 21
#define PIN_MOTOR_2 20
#define PIN_MOTOR_3 19
#define PIN_MOTOR_4 18
#define PIN_MOTOR_6 17
#define PWM_FREQ 50
#define PWM_PERIOD_MS 1000*(1/PWM_FREQ)

#define MOTOR_1 0
#define MOTOR_2 1
#define MOTOR_3 2
#define MOTOR_4 3
#define MOTOR_6 4

#define N 4

#define SYS_FREQ_KHZ 250000

volatile bool claw_position = true;

void pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, float d);
void multiply_matrices(float first[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float second[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float result[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
void display(float result[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
void transform_matrix(float alpha, float a, float d, float theta, float T[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
void forward_kinematics(float theta1, float theta2, float theta3, float theta4, float xyzpitch[4]);
void jacobian_function(float theta1, float theta2, float theta3, float theta4, float jacobian[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
float map_function(float input, float input_start, float input_end, float output_start, float output_end);
void motor_move(uint slice_motors[], uint chan_motors[]);
void getCofactor(float mat[N][N], float temp[N][N], int p, int q, int n);
float determinant(float mat[N][N], int n);
void adjoint(float mat[N][N], float adj[N][N]);
int inverse(float mat[N][N], float inverse[N][N]);
void claw_move(uint slice_motors[], uint chan_motors[]);

struct pwmSignal{
    float motor1;
    float motor2;
    float motor3;
    float motor4;
    float motor6;
};

struct pwmSignal duty_cycle_motors;

void gpio_callback(uint gpio, uint32_t events) {
    claw_position = !claw_position;
}

// Function to get cofactor of mat[p][q] in temp[][]. n is the current dimension of mat[][]
void getCofactor(float mat[N][N], float temp[N][N], int p, int q, int n) {
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
float determinant(float mat[N][N], int n) {
    float D = 0;  // Initialize result

    // Base case: if matrix contains single element
    if (n == 1)
        return mat[0][0];

    float temp[N][N];  // To store cofactors
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

// Function to get adjoint of mat[N][N] in adj[N][N]
void adjoint(float mat[N][N], float adj[N][N]) {
    if (N == 1) {
        adj[0][0] = 1;
        return;
    }

    // temp is used to store cofactors
    int sign = 1;
    float temp[N][N];

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            // Get cofactor of mat[i][j]
            getCofactor(mat, temp, i, j, N);

            // Sign of adj[j][i] positive if sum of row and column indexes is even.
            sign = ((i + j) % 2 == 0) ? 1 : -1;

            // Interchanging rows and columns to get the transpose of the cofactor matrix
            adj[j][i] = (sign) * (determinant(temp, N - 1));
        }
    }
}

// Function to calculate the inverse of a matrix. Returns 1 if the matrix is invertible, otherwise 0.
int inverse(float mat[N][N], float inverse[N][N]) {
    // Find determinant of matrix
    float det = determinant(mat, N);
    if (det == 0) {
        printf("Matrix is singular, cannot find its inverse\n");
        return 0;
    }

    // Find adjoint
    float adj[N][N];
    adjoint(mat, adj);

    // Inverse is adjoint divided by determinant
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            inverse[i][j] = adj[i][j] / det;

    return 1;
}

void pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, float d)
{
    uint32_t clock = SYS_FREQ_KHZ * 1000;
    uint16_t wrapval = 65535;
    uint16_t chan_level = wrapval * d;
    float clkdiv = (double)clock/(wrapval * f);
    pwm_set_clkdiv(slice_num, clkdiv);
    pwm_set_wrap(slice_num, wrapval);
    pwm_set_chan_level(slice_num, chan, chan_level);
}

// function to multiply two matrices
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

void transform_matrix(float alpha, float a, float d, float theta, float T[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]) {
    T[0][0] = cos(theta);
    T[0][1] = -sin(theta);
    T[0][2] = 0;
    T[0][3] = a;

    T[1][0] = sin(theta) * cos(alpha);
    T[1][1] = cos(theta) * cos(alpha);
    T[1][2] = -sin(alpha);
    T[1][3] = -d * sin(alpha);

    T[2][0] = sin(theta) * sin(alpha);
    T[2][1] = cos(theta) * sin(alpha);
    T[2][2] = cos(alpha);
    T[2][3] = d * cos(alpha);

    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;
}

void forward_kinematics(float theta1, float theta2, float theta3, float theta4, float xyzpitch[4]){
    float alpha[5] = {0, M_PI/2, 0, 0, 0};
    float a[5] = {0, 0, DISTANCE_LINK_2, DISTANCE_LINK_3, DISTANCE_LINK_4};
    float d[5] = {DISTANCE_LINK_1, 0, 0, 0, 0};
    float Q[5] = {theta1, theta2, theta3, theta4, 0};

    float T01[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T12[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T23[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T34[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T45[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];

    float T02[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T24[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T25[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];
    float T05[MATRIX_ROW_SIZE][MATRIX_COL_SIZE];

    transform_matrix(alpha[0], a[0], d[0], Q[0], T01);
    transform_matrix(alpha[1], a[1], d[1], Q[1], T12);
    transform_matrix(alpha[2], a[2], d[2], Q[2], T23);
    transform_matrix(alpha[3], a[3], d[3], Q[3], T34);
    transform_matrix(alpha[4], a[4], d[4], Q[4], T45);

    multiply_matrices(T01, T12, T02);
    multiply_matrices(T23, T34, T24);
    multiply_matrices(T24, T45, T25);
    multiply_matrices(T02, T25, T05);

    xyzpitch[0] = T05[0][3];
    xyzpitch[1] = T05[1][3];
    xyzpitch[2] = T05[2][3];
    xyzpitch[3] = atanf(-T05[2][0]/(sqrtf(powf(T05[0][0],2) + powf(T05[1][0],2))));
}

void jacobian_function(float theta1, float theta2, float theta3, float theta4, float jacobian[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]){
    float s1 = sin(theta1);
    float s2 = sin(theta2);
    float s3 = sin(theta3);
    float s4 = sin(theta4);
    float c1 = cos(theta1);
    float c2 = cos(theta2);
    float c3 = cos(theta3);
    float c4 = cos(theta4);

    float J11 = (-s1*c2)*(DISTANCE_LINK_4*c3*c4 - DISTANCE_LINK_4*s3*s4 + DISTANCE_LINK_3*c3 + DISTANCE_LINK_2) + (s1*s2)*(DISTANCE_LINK_4*s3*c4 + DISTANCE_LINK_4*c3*s4 + DISTANCE_LINK_3*s3);
    float J12 = (-s2*c1)*(DISTANCE_LINK_4*c3*c4 - DISTANCE_LINK_4*s3*s4 + DISTANCE_LINK_3*c3 + DISTANCE_LINK_2) - (c1*c2)*(DISTANCE_LINK_4*s3*c4 + DISTANCE_LINK_4*c3*s4 + DISTANCE_LINK_3*s3);
    float J13 = (c1*c2)*(-DISTANCE_LINK_4*s3*c4 - DISTANCE_LINK_4*c3*s4 - DISTANCE_LINK_3*s3) + (-c1*s2)*(DISTANCE_LINK_4*c3*c4 - DISTANCE_LINK_4*s3*s4 + DISTANCE_LINK_3*c3);
    float J14 = (c1*c2)*(-DISTANCE_LINK_4*c3*s4 - DISTANCE_LINK_4*s3*c4) + (-c1*s2)*(-DISTANCE_LINK_4*s3*s4 + DISTANCE_LINK_4*c3*c4);
    float J21 = (c1*c2)*(DISTANCE_LINK_4*c3*c4 - DISTANCE_LINK_4*s3*s4 + DISTANCE_LINK_3*c3 + DISTANCE_LINK_2) - (c1*s2)*(DISTANCE_LINK_4*s3*c4 + DISTANCE_LINK_4*c3*s4 + DISTANCE_LINK_3*s3);
    float J22 = (-s1*s2)*(DISTANCE_LINK_4*c3*c4 - DISTANCE_LINK_4*s3*s4 + DISTANCE_LINK_3*c3 + DISTANCE_LINK_2) - (s1*c2)*(DISTANCE_LINK_4*s3*c4 + DISTANCE_LINK_4*c3*s4 + DISTANCE_LINK_3*s3);
    float J23 = (s1*c2)*(-DISTANCE_LINK_4*s3*c4 - DISTANCE_LINK_4*c3*s4 - DISTANCE_LINK_3*s3) - (s1*s2)*(DISTANCE_LINK_4*c3*c4 - DISTANCE_LINK_4*s3*s4 + DISTANCE_LINK_3*c3);
    float J24 = (s1*c2)*(-DISTANCE_LINK_4*c3*s4 - DISTANCE_LINK_4*s3*c4) - (s1*s2)*(-DISTANCE_LINK_4*s3*s4 + DISTANCE_LINK_4*c3*c4);
    float J31 = 0;
    float J32 = c2*(DISTANCE_LINK_4*c3*c4 - DISTANCE_LINK_4*s3*s4 + DISTANCE_LINK_3*c3 + DISTANCE_LINK_2) - s2*(DISTANCE_LINK_4*s3*c4 + DISTANCE_LINK_4*c3*s4 + DISTANCE_LINK_3*s3);
    float J33 = s2*(-DISTANCE_LINK_4*s3*c4 - DISTANCE_LINK_4*c3*s4 - DISTANCE_LINK_3*s3) + c2*(DISTANCE_LINK_4*c3*c4 - DISTANCE_LINK_4*s3*s4 + DISTANCE_LINK_3*c3);
    float J34 = s2*(-DISTANCE_LINK_4*c3*s4 - DISTANCE_LINK_4*s3*c4) + c2*(-DISTANCE_LINK_4*s3*s4 + DISTANCE_LINK_4*c3*c4);

    float J41 = 0;
    float J42 = -c1;
    float J43 = -c1;
    float J44 = -c1;

    jacobian[0][0] = J11;
    jacobian[0][1] = J12;
    jacobian[0][2] = J13;
    jacobian[0][3] = J14;

    jacobian[1][0] = J21;
    jacobian[1][1] = J22;
    jacobian[1][2] = J23;
    jacobian[1][3] = J24;

    jacobian[2][0] = J31;
    jacobian[2][1] = J32;
    jacobian[2][2] = J33;
    jacobian[2][3] = J34;

    jacobian[3][0] = J41;
    jacobian[3][1] = J42;
    jacobian[3][2] = J43;
    jacobian[3][3] = J44;
}

float map_function(float input, float input_start, float input_end, float output_start, float output_end){
    float slope = (output_end - output_start) / (input_end - input_start);
    float output = output_start + slope * (input - input_start);

    return output;
}

void duty_cycle_set(float theta1, float theta2, float theta3, float theta4, uint config){
    duty_cycle_motors.motor1 = map_function(theta1, -M_PI/2, M_PI/2, 0.5, 2.5)/20;
    duty_cycle_motors.motor2 = map_function(theta2, M_PI, 0, 0.5, 2.5)/20;
    duty_cycle_motors.motor3 = map_function(theta3, (-3*M_PI)/4, 0, 0.56, 2.06)/20;
    duty_cycle_motors.motor4 = map_function(theta4, -M_PI/2, M_PI/2, 0.5, 2.5)/20;
}

void motor_move(uint slice_motors[], uint chan_motors[]){
    pwm_set_freq_duty(slice_motors[MOTOR_1], chan_motors[MOTOR_1], PWM_FREQ, duty_cycle_motors.motor1);
    pwm_set_freq_duty(slice_motors[MOTOR_2], chan_motors[MOTOR_2], PWM_FREQ, duty_cycle_motors.motor2);
    pwm_set_freq_duty(slice_motors[MOTOR_3], chan_motors[MOTOR_3], PWM_FREQ, duty_cycle_motors.motor3);
    pwm_set_freq_duty(slice_motors[MOTOR_4], chan_motors[MOTOR_4], PWM_FREQ, duty_cycle_motors.motor4);

    pwm_set_enabled(slice_motors[MOTOR_1], true);
    pwm_set_enabled(slice_motors[MOTOR_2], true);
    pwm_set_enabled(slice_motors[MOTOR_3], true);
    pwm_set_enabled(slice_motors[MOTOR_4], true);
}

float norm(float position_difference[4]){
    float norm = 0;
    norm = sqrtf(powf(position_difference[0], 2) + powf(position_difference[1], 2) + powf(position_difference[2], 2) + powf(position_difference[3], 2));
    return norm;
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

void calculate_velocity(float position_difference[4],float velocity[4][1],float speed){
    for(int i=0; i<4; ++i){
        velocity[i][0] = speed*(position_difference[i]/norm(position_difference));
    }
}

void claw_move(uint slice_motors[], uint chan_motors[]){
    if(claw_position == true){
        pwm_set_freq_duty(slice_motors[MOTOR_6], chan_motors[MOTOR_6], PWM_FREQ, 2.15/20);
    }
    else{
        pwm_set_freq_duty(slice_motors[MOTOR_6], chan_motors[MOTOR_6], PWM_FREQ, 1.4/20);
    }
    pwm_set_enabled(slice_motors[MOTOR_6], true);
}

void robot_move(float xyzpitch[4],uint slice_motors[],uint chan_motors[]){
    float theta1 = map_function(20*duty_cycle_motors.motor1, 0.5, 2.5, -M_PI/2, M_PI/2);
    float theta2 = map_function(20*duty_cycle_motors.motor2, 0.5, 2.5, M_PI, 0);
    float theta3 = map_function(20*duty_cycle_motors.motor3, 0.56, 2.06, (-3*M_PI)/4, 0);
    float theta4 = map_function(20*duty_cycle_motors.motor4, 0.5, 2.5, -M_PI/2, M_PI/2);

    float position_final[4];
    float position_initial[4];
    float initial_angle[4];
    float jacobian_matrix[4][4];
    float jacobian_inverse[4][4];
    float position_difference[4] = {1, 1, 1, 1};
    float total_distance[4];
    float velocity[4][1];
    float new_angle[4];
    float time_step = 0.01;
    float delta_angle[4][1];
    float error = 0.01;

    uint32_t count = 0;
    float speed = 1;

    jacobian_function(theta1, theta2, theta3, theta4, jacobian_matrix);
    forward_kinematics(theta1, theta2, theta3, theta4, position_initial);

    for(int i=0; i<4; ++i){
        position_final[i] = xyzpitch[i];
    }

    for(int i=0; i<4; ++i){
        total_distance[i] = position_final[i] - position_initial[i];
    }

    initial_angle[0] = theta1;
    initial_angle[1] = theta2;
    initial_angle[2] = theta3;
    initial_angle[3] = theta4;

    uint64_t start;
    uint64_t end;
    while(norm(position_difference) >= error){
        add_subtract_matrix(position_final, position_initial, position_difference, false);

        calculate_velocity(position_difference, velocity, speed);

        if(inverse(jacobian_matrix, jacobian_inverse) == 0){
            error = 4000;
        }

        multiply_matrices_angle(jacobian_inverse, velocity, delta_angle);

        initial_angle[0] = delta_angle[0][0]*time_step + initial_angle[0];
        initial_angle[1] = delta_angle[1][0]*time_step + initial_angle[1];
        initial_angle[2] = delta_angle[2][0]*time_step + initial_angle[2];
        initial_angle[3] = delta_angle[3][0]*time_step + initial_angle[3];

        duty_cycle_set(initial_angle[0], initial_angle[1], initial_angle[2], initial_angle[3], 1);
        motor_move(slice_motors, chan_motors);

        forward_kinematics(initial_angle[0], initial_angle[1], initial_angle[2], initial_angle[3], position_initial);
        jacobian_function(initial_angle[0], initial_angle[1], initial_angle[2], initial_angle[3], jacobian_matrix);

        ++count;
        if(count >= 500){error = 4000;}
    }
}

int main()
{
    set_sys_clock_khz(SYS_FREQ_KHZ, true);
    stdio_init_all();

    // PWM initialization to control motors
    gpio_set_function(PIN_MOTOR_1, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_2, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_3, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_4, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_6, GPIO_FUNC_PWM);

    uint slice_motors[NUMBER_OF_MOTORS];
    uint chan_motors[NUMBER_OF_MOTORS];

    slice_motors[MOTOR_1] = pwm_gpio_to_slice_num(PIN_MOTOR_1);
    slice_motors[MOTOR_2] = pwm_gpio_to_slice_num(PIN_MOTOR_2);
    slice_motors[MOTOR_3] = pwm_gpio_to_slice_num(PIN_MOTOR_3);
    slice_motors[MOTOR_4] = pwm_gpio_to_slice_num(PIN_MOTOR_4);
    slice_motors[MOTOR_6] = pwm_gpio_to_slice_num(PIN_MOTOR_6);

    chan_motors[MOTOR_1] = pwm_gpio_to_channel(PIN_MOTOR_1);
    chan_motors[MOTOR_2] = pwm_gpio_to_channel(PIN_MOTOR_2);
    chan_motors[MOTOR_3] = pwm_gpio_to_channel(PIN_MOTOR_3);
    chan_motors[MOTOR_4] = pwm_gpio_to_channel(PIN_MOTOR_4);
    chan_motors[MOTOR_6] = pwm_gpio_to_channel(PIN_MOTOR_6);

    duty_cycle_set(0, M_PI/2, -M_PI/2, 0, 1);
    motor_move(slice_motors, chan_motors);
    claw_move(slice_motors, chan_motors);
    sleep_ms(1000);

    //pwm_set_enabled
    /* I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c */

    //Initialize ADC
    adc_init();
    adc_gpio_init(28);
    adc_gpio_init(27);
    adc_gpio_init(26);

    //Initialize interrupts
    gpio_set_irq_enabled_with_callback(1, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);


    float T1[MATRIX_ROW_SIZE][MATRIX_COL_SIZE] = {0};  
    float xyzpitch[4] = {0};
    float jacobian[MATRIX_ROW_SIZE][MATRIX_COL_SIZE] = {0};
    float jacobian_inverse[MATRIX_ROW_SIZE][MATRIX_COL_SIZE] = {0};

    uint64_t start;
    uint64_t end;
    uint64_t time_taken;

    const float conversion_factor = 3.3f / (1 << 12);
    uint16_t result;
    float countY = 0;
    float countX = 0;
    float countZ = 0;
    const float starting_x = 25;
    const float starting_y = 0;
    const float starting_z = 19.9;
    const float starting_pitch = 0;

    float diff_x;
    float diff_y;
    float diff_z;
    
    while (true) {
    /*
    xyzpitch[0] = 28;
    xyzpitch[1] = -10;
    xyzpitch[2] = 18;
    xyzpitch[3] = 0;
    robot_move(xyzpitch, slice_motors, chan_motors);
    */
        /*
        multiply_matrices_angle(matrix1, matrix2, result);
        printf("%f \n", result[0][0]);
        printf("%f \n", result[1][0]);
        printf("%f \n", result[2][0]);
        printf("%f \n\n", result[3][0]);
        */
        diff_x = xyzpitch[0];
        diff_y = xyzpitch[1];
        diff_z = xyzpitch[2];

        adc_select_input(0);
        result = adc_read();
        if(result < 1500){
            countY += 0.03;
        }
        if(result > 2500){
            countY -= 0.03;
        }
        adc_select_input(1);
        result = adc_read();
        if(result < 1900){
            countZ += 0.02;
        }
        if(result > 2100){
            countZ -= 0.02;
        }
        adc_select_input(2);
        result = adc_read();
        if(result < 1500){
            countX += 0.03;
        }
        if(result > 2500){
            countX -= 0.03;
        }
        
        xyzpitch[0] = starting_x + countX;
        xyzpitch[1] = starting_y + countY;
        xyzpitch[2] = starting_z + countZ;
        xyzpitch[3] = starting_pitch;
        
        if(xyzpitch[0] != diff_x || xyzpitch[1] != diff_y || xyzpitch[2] != diff_z){
            robot_move(xyzpitch, slice_motors, chan_motors);
        }

        claw_move(slice_motors, chan_motors);
   //     float first[4][4] = {{1,1,1,1},{2,2,2,2},{3,3,3,3},{4,4,4,4}};
     //   float second[4][4] = {{1,1,1,1},{2,2,2,2},{3,3,3,3},{4,4,4,4}};
    //    float result[4][4] = {0};
     //   multiply_matrices(first, second, result);
        
     //   printf("%llu \n", start);
      //  printf("%llu \n", end);
       
    
    }
}

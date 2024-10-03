#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

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
#define DISTANCE_LINK_3 8.9 
#define DISTANCE_LINK_4 16.7

#define NUMBER_OF_MOTORS 5
#define PIN_MOTOR_1 17
#define PIN_MOTOR_2 18
#define PIN_MOTOR_3 19
#define PIN_MOTOR_4 20
#define PIN_MOTOR_6 21
#define PWM_FREQ 50
#define PWM_PERIOD_MS 1000*(1/PWM_FREQ)

#define MOTOR_1 0
#define MOTOR_2 1
#define MOTOR_3 2
#define MOTOR_4 3
#define MOTOR_6 4

#define SYS_FREQ_KHZ 250000

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, float d);
void multiply_matrices(float first[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float second[MATRIX_ROW_SIZE][MATRIX_COL_SIZE],float result[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
void display(float result[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
void transform_matrix(float alpha, float a, float d, float theta, float T[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
void forward_kinematics(float theta1, float theta2, float theta3, float theta4, float xyzpitch[4]);
void jacobian_function(float theta1, float theta2, float theta3, float theta4, float jacobian[MATRIX_ROW_SIZE][MATRIX_COL_SIZE]);
float map_function(float input, float input_start, float input_end, float output_start, float output_end);
void motor_move(uint slice_motors[], uint chan_motors[]);

struct pwmSignal{
    float motor1;
    float motor2;
    float motor3;
    float motor4;
    float motor6;
};

struct pwmSignal duty_cycle_motors;

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, float d)
{
    uint32_t clock = SYS_FREQ_KHZ;
    uint16_t wrapval = 65535;
    uint16_t chan_level = wrapval * d;
    pwm_set_clkdiv(slice_num, 76.3);
    pwm_set_wrap(slice_num, 65535);
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
    pwm_set_enabled(slice_motors[MOTOR_6], true);
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

    chan_motors[MOTOR_1] = pwm_gpio_to_slice_num(PIN_MOTOR_1);
    chan_motors[MOTOR_2] = pwm_gpio_to_slice_num(PIN_MOTOR_2);
    chan_motors[MOTOR_3] = pwm_gpio_to_slice_num(PIN_MOTOR_3);
    chan_motors[MOTOR_4] = pwm_gpio_to_slice_num(PIN_MOTOR_4);
    chan_motors[MOTOR_6] = pwm_gpio_to_slice_num(PIN_MOTOR_6);

    duty_cycle_set(0, M_PI/2, -M_PI/2, 0, 1);
    motor_move(slice_motors, chan_motors);

    //pwm_set_enabled
    /* I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c */

    float T1[MATRIX_ROW_SIZE][MATRIX_COL_SIZE] = {0};  
    float xyzpitch[4] = {0};
    float jacobian[MATRIX_ROW_SIZE][MATRIX_COL_SIZE] = {0};

    uint64_t start;
    uint64_t end;
    uint64_t time_taken;
    while (true) {
        printf("%f \n", duty_cycle_motors.motor1);
        sleep_ms(1000);


   //     float first[4][4] = {{1,1,1,1},{2,2,2,2},{3,3,3,3},{4,4,4,4}};
     //   float second[4][4] = {{1,1,1,1},{2,2,2,2},{3,3,3,3},{4,4,4,4}};
    //    float result[4][4] = {0};
     //   multiply_matrices(first, second, result);
        
     //   printf("%llu \n", start);
      //  printf("%llu \n", end);
       
    
    }
}

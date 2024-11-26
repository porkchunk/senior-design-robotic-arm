#include "robot_commands.h"
#include "motor.h"
#include "pico/stdlib.h"
#include "main.h"
#include "pwm.h"
#include "robot_modes.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float xyzpitch[4];

volatile bool claw_position = true;

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
    xyzpitch[0] = DISTANCE_LINK_4*(cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) + cos(theta1)*cos(theta2)*(DISTANCE_LINK_2 + DISTANCE_LINK_3*cos(theta3)) - DISTANCE_LINK_3*cos(theta1)*sin(theta2)*sin(theta3);
    xyzpitch[1] = DISTANCE_LINK_4*(cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3))) + cos(theta2)*sin(theta1)*(DISTANCE_LINK_2 + DISTANCE_LINK_3*cos(theta3)) - DISTANCE_LINK_3*sin(theta1)*sin(theta2)*sin(theta3);
    xyzpitch[2] = DISTANCE_LINK_1 + sin(theta2)*(DISTANCE_LINK_2 + DISTANCE_LINK_3*cos(theta3)) + DISTANCE_LINK_4*(cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4))) + DISTANCE_LINK_3*cos(theta2)*sin(theta3);
    
    float R31 = cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4));
    float R11 = cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3));
    float R21 = cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3));
 
    xyzpitch[3] = -asinf(R31); //atanf(-R31/(sqrtf(powf(R11,2) + powf(R21,2)))); //;
    /*
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
    */
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

void claw_move(bool narrow){
    if(claw_position == true){
        pwm_set_freq_duty(slice_motors[MOTOR_6], chan_motors[MOTOR_6], PWM_FREQ, 2.15/20);
    }
    else if((claw_position == false) && (narrow == true)){
        pwm_set_freq_duty(slice_motors[MOTOR_6], chan_motors[MOTOR_6], PWM_FREQ, 1.95/20);
    }
    else{
        pwm_set_freq_duty(slice_motors[MOTOR_6], chan_motors[MOTOR_6], PWM_FREQ, 1.4/20);
    }
    pwm_set_enabled(slice_motors[MOTOR_6], true);
}

void set_zero_position(){
    duty_cycle_set(0, M_PI/2, -M_PI/2, 0, 1);
    motor_move(slice_motors, chan_motors);
    claw_position = false;
    claw_move(false);

    sleep_ms(1000);
}

void set_initial_position(){
    forward_kinematics(0, 2*M_PI/3, -2*M_PI/3, 0, xyzpitch);
    robot_move(xyzpitch, 2);
    //forward_kinematics(0, 2*M_PI/3, -2*M_PI/3, 0 , xyzpitch);
   // robot_move(xyzpitch);
}

void robot_move(float xyzpitch[4], float speed){
    //Convert current duty_cycle to angle based on model of robot
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
    float delta_angle[4][1];
    float time_step = 0.0025; //make 0.00033
    float error = 0.01;

    uint32_t count = 0;

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

    while(norm(position_difference) >= error){
        uint64_t start = time_us_64();
        //position_final = position_initial - position_difference
        add_subtract_matrix(position_final, position_initial, position_difference, false);

        //velocity = speed*(position_difference/norm(position_difference))
        calculate_velocity(position_difference, velocity, speed);

        //Exit if jacobian_matrix is not invertible
        inverse(4, jacobian_matrix, jacobian_inverse);

        //Calculate delta_angle from delta_angle = J^{-1} * Velocity
        multiply_matrices_angle(jacobian_inverse, velocity, delta_angle);

        //Calculate all new angles to send to motors
        initial_angle[0] = delta_angle[0][0]*time_step + initial_angle[0];
        initial_angle[1] = delta_angle[1][0]*time_step + initial_angle[1];
        initial_angle[2] = delta_angle[2][0]*time_step + initial_angle[2];
        initial_angle[3] = delta_angle[3][0]*time_step + initial_angle[3];

        //Send new angles to motors
        duty_cycle_set(initial_angle[0], initial_angle[1], initial_angle[2], initial_angle[3], 1);
        motor_move(slice_motors, chan_motors);

        //Calculate new position and jacobian
        forward_kinematics(initial_angle[0], initial_angle[1], initial_angle[2], initial_angle[3], position_initial);
        jacobian_function(initial_angle[0], initial_angle[1], initial_angle[2], initial_angle[3], jacobian_matrix);

        //End robot_move if count goes too high
        uint64_t end = time_us_64();
        //printf("%llu \n", end - start);
        ++count;
        if(count >= norm(total_distance)/(speed*time_step)){error = 4000;}
    }
}
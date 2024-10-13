#include "motor.h"
#include "pico/stdlib.h"
#include "pwm.h"
#include "robot_commands.h"
#include <math.h>

void duty_cycle_set(float theta1, float theta2, float theta3, float theta4, uint config){
    duty_cycle_motors.motor1 = map_function(theta1, -M_PI/2, M_PI/2, 0.5, 2.5)/20;
    duty_cycle_motors.motor2 = map_function(theta2, M_PI, 0, 0.5, 2.5)/20;
    duty_cycle_motors.motor3 = map_function(theta3, (-3*M_PI)/4, 0, 0.56, 2.06)/20;
    duty_cycle_motors.motor4 = map_function(theta4, -M_PI/2, M_PI/2, 0.5, 2.5)/20;
}

void motor_move(){
    pwm_set_freq_duty(slice_motors[MOTOR_1], chan_motors[MOTOR_1], PWM_FREQ, duty_cycle_motors.motor1);
    pwm_set_freq_duty(slice_motors[MOTOR_2], chan_motors[MOTOR_2], PWM_FREQ, duty_cycle_motors.motor2);
    pwm_set_freq_duty(slice_motors[MOTOR_3], chan_motors[MOTOR_3], PWM_FREQ, duty_cycle_motors.motor3);
    pwm_set_freq_duty(slice_motors[MOTOR_4], chan_motors[MOTOR_4], PWM_FREQ, duty_cycle_motors.motor4);
    pwm_set_freq_duty(slice_motors[MOTOR_3], chan_motors[MOTOR_5], PWM_FREQ, duty_cycle_motors.motor3);
    pwm_set_freq_duty(slice_motors[MOTOR_4], chan_motors[MOTOR_6], PWM_FREQ, duty_cycle_motors.motor4);

    pwm_set_enabled(slice_motors[MOTOR_1], true);
    pwm_set_enabled(slice_motors[MOTOR_2], true);
    pwm_set_enabled(slice_motors[MOTOR_3], true);
    pwm_set_enabled(slice_motors[MOTOR_4], true);
}


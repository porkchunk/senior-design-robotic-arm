#include "motor.h"
#include "pico/stdlib.h"
#include "robot_commands.h"
#include <math.h>

uint chan_motors[NUMBER_OF_MOTORS];
uint slice_motors[NUMBER_OF_MOTORS];
struct pwmSignal duty_cycle_motors;

void motor_move(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6, uint config){
    duty_cycle_motors.motor1 = map_function(theta1, -M_PI/2, M_PI/2, 0.5, 2.5)/20;
    duty_cycle_motors.motor2 = map_function(theta2, M_PI, 0, 0.5, 2.5)/20;
    duty_cycle_motors.motor3 = map_function(theta3, (-3*M_PI)/4, 0, 0.56, 2.06)/20;
    duty_cycle_motors.motor4 = map_function(theta4, -M_PI/2, M_PI/2, 0.5, 2.5)/20;
    duty_cycle_motors.motor5 = map_function(theta3, (-3*M_PI)/4, 0, 0.56, 2.06)/20;
    duty_cycle_motors.motor6 = map_function(theta4, -M_PI/2, M_PI/2, 0.5, 2.5)/20;

    pwm_set_freq_duty(slice_motors[MOTOR_1], chan_motors[MOTOR_1], PWM_FREQ, duty_cycle_motors.motor1);
    pwm_set_freq_duty(slice_motors[MOTOR_2], chan_motors[MOTOR_2], PWM_FREQ, duty_cycle_motors.motor2);
    pwm_set_freq_duty(slice_motors[MOTOR_3], chan_motors[MOTOR_3], PWM_FREQ, duty_cycle_motors.motor3);
    pwm_set_freq_duty(slice_motors[MOTOR_4], chan_motors[MOTOR_4], PWM_FREQ, duty_cycle_motors.motor4);
    pwm_set_freq_duty(slice_motors[MOTOR_5], chan_motors[MOTOR_5], PWM_FREQ, duty_cycle_motors.motor5);
    pwm_set_freq_duty(slice_motors[MOTOR_6], chan_motors[MOTOR_6], PWM_FREQ, duty_cycle_motors.motor6);

    pwm_set_enabled(slice_motors[MOTOR_1], true);
    pwm_set_enabled(slice_motors[MOTOR_2], true);
    pwm_set_enabled(slice_motors[MOTOR_3], true);
    pwm_set_enabled(slice_motors[MOTOR_4], true);
    pwm_set_enabled(slice_motors[MOTOR_5], true);
    pwm_set_enabled(slice_motors[MOTOR_6], true);
}

void pwm_initialization(){
    gpio_set_function(PIN_MOTOR_1, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_2, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_3, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_4, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_5, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_6, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_7, GPIO_FUNC_PWM);

    slice_motors[MOTOR_1] = pwm_gpio_to_slice_num(PIN_MOTOR_1);
    slice_motors[MOTOR_2] = pwm_gpio_to_slice_num(PIN_MOTOR_2);
    slice_motors[MOTOR_3] = pwm_gpio_to_slice_num(PIN_MOTOR_3);
    slice_motors[MOTOR_4] = pwm_gpio_to_slice_num(PIN_MOTOR_4);
    slice_motors[MOTOR_6] = pwm_gpio_to_slice_num(PIN_MOTOR_5);
    slice_motors[MOTOR_4] = pwm_gpio_to_slice_num(PIN_MOTOR_6);
    slice_motors[MOTOR_6] = pwm_gpio_to_slice_num(PIN_MOTOR_7);

    chan_motors[MOTOR_1] = pwm_gpio_to_channel(PIN_MOTOR_1);
    chan_motors[MOTOR_2] = pwm_gpio_to_channel(PIN_MOTOR_2);
    chan_motors[MOTOR_3] = pwm_gpio_to_channel(PIN_MOTOR_3);
    chan_motors[MOTOR_4] = pwm_gpio_to_channel(PIN_MOTOR_4);
    chan_motors[MOTOR_6] = pwm_gpio_to_channel(PIN_MOTOR_5);
    chan_motors[MOTOR_4] = pwm_gpio_to_channel(PIN_MOTOR_6);
    chan_motors[MOTOR_6] = pwm_gpio_to_channel(PIN_MOTOR_7);
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


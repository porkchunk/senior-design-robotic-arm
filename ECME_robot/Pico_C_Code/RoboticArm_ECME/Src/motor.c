#include "motor.h"
#include "pico/stdlib.h"
#include "robot_commands.h"
#include "SPI.h"
#include <math.h>

uint chan_motors[NUMBER_OF_MOTORS];
uint slice_motors[NUMBER_OF_MOTORS];
struct pwmSignal duty_cycle_motors;

uint32_t current_time = 0;
uint32_t delta_time = 0;
float previous_time = 0;
float error_value[6] = {0};
float previous_error[6]= {0};
float actual_position[6] = {0};
float target_position[6] = {0};
float error_derivative[6] = {0};
float error_integral[6] = {0};
float control_signal[6] = {0};

//Add direction pins here
const int PIN_DIRECTION[6] = {0};

//PID terms for user to adjust
float proportional[6] = {0};
float integral[6] = {0};
float derivative[6] = {0};

void calculate_PID(){
    current_time = time_us_32();
    delta_time = (current_time - previous_time) / 1e6;
    previous_time = current_time;

    for(int i=0; i<6; ++i){
        error_value[i] = actual_position[i] - target_position[i];
        error_derivative[i] = (error_value[i] - previous_error[i]) / delta_time;
        error_integral[i] = error_integral[i] + (error_value[i] * delta_time);
        control_signal[i] = (proportional[i] * error_value[i]) + (derivative[i] * error_derivative[i]) + (integral[i] * error_integral[i]);
        previous_error[i] = error_value[i];
    }
}

void motor_move(){
    //Set direction of motors based on error
    for(int i=0; i<6; ++i){
        if(control_signal[i] < 0){
            gpio_put(PIN_DIRECTION[i], true);
        }
        else if (control_signal[i] > 0){
            gpio_put(PIN_DIRECTION[i], false);
        }
    }

    for(int i=0; i<6; ++i){
        if(control_signal[i] > 1){
            control_signal[i] = 1;
        }
        if(control_signal[i] < 0.275 && error_value[i] != 0){
            control_signal[i] = 0.275;
        }
    }
    
    duty_cycle_motors.motor1 = control_signal[MOTOR_1];
    duty_cycle_motors.motor2 = control_signal[MOTOR_2];
    duty_cycle_motors.motor3 = control_signal[MOTOR_3];
    duty_cycle_motors.motor4 = control_signal[MOTOR_4];
    duty_cycle_motors.motor5 = control_signal[MOTOR_5];
    duty_cycle_motors.motor6 = control_signal[MOTOR_6];
    
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

void claw_move(){
    if(claw_position == true){
        pwm_set_freq_duty(slice_motors[MOTOR_7], chan_motors[MOTOR_7], PWM_FREQ, 2.15/20);
    }
    else{
        pwm_set_freq_duty(slice_motors[MOTOR_7], chan_motors[MOTOR_7], PWM_FREQ, 1.4/20);
    }
    pwm_set_enabled(slice_motors[MOTOR_7], true);
}

void motor_initialization(){
    for(int i=0; i<6; ++i){
        gpio_init(PIN_DIRECTION[i]);
        gpio_set_dir(PIN_DIRECTION[i], GPIO_OUT);
    }

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
    slice_motors[MOTOR_5] = pwm_gpio_to_slice_num(PIN_MOTOR_5);
    slice_motors[MOTOR_6] = pwm_gpio_to_slice_num(PIN_MOTOR_6);
    slice_motors[MOTOR_7] = pwm_gpio_to_slice_num(PIN_MOTOR_7);

    chan_motors[MOTOR_1] = pwm_gpio_to_channel(PIN_MOTOR_1);
    chan_motors[MOTOR_2] = pwm_gpio_to_channel(PIN_MOTOR_2);
    chan_motors[MOTOR_3] = pwm_gpio_to_channel(PIN_MOTOR_3);
    chan_motors[MOTOR_4] = pwm_gpio_to_channel(PIN_MOTOR_4);
    chan_motors[MOTOR_5] = pwm_gpio_to_channel(PIN_MOTOR_5);
    chan_motors[MOTOR_6] = pwm_gpio_to_channel(PIN_MOTOR_6);
    chan_motors[MOTOR_7] = pwm_gpio_to_channel(PIN_MOTOR_7);
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


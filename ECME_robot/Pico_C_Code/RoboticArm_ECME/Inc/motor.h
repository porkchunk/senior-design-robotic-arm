#ifndef MOTOR_H
#define MOTOR_H

#define MOTOR_1 0
#define MOTOR_2 1
#define MOTOR_3 2
#define MOTOR_4 3 
#define MOTOR_5 4 
#define MOTOR_6 5
#define MOTOR_7 6

#define NUMBER_OF_MOTORS 7
#define PIN_MOTOR_1 21
#define PIN_MOTOR_2 20
#define PIN_MOTOR_3 19
#define PIN_MOTOR_4 18 
#define PIN_MOTOR_5 16 
#define PIN_MOTOR_6 17
#define PIN_MOTOR_7 22

#define DIRECTION_MOTOR_1 8
#define DIRECTION_MOTOR_3 7
#define DIRECTION_MOTOR_4 6 
#define DIRECTION_MOTOR_5 5 
#define DIRECTION_MOTOR_6 4

#include "pico/stdlib.h"
#include "main.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

#define PWM_FREQ 50
#define PWM_PERIOD_MS 1000*(1/PWM_FREQ)

//add direction pins here
extern const int PIN_DIRECTION[6];

struct pwmSignal{
    float motor1;
    float motor2;
    float motor3;
    float motor4;
    float motor5;
    float motor6;
    float motor7;
};

extern struct pwmSignal duty_cycle_motors;
extern uint slice_motors[NUMBER_OF_MOTORS];
extern uint chan_motors[NUMBER_OF_MOTORS];
extern int target_position[6];
extern int actual_position[6];
extern int motor_number;

void pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, float d);
void motor_initialization();
void motor_move();
void calculate_PID();
void pwm_initialization();
void set_pin_directions();
void update_motor_speed();
void calculate_control_signals();
void move_motor_with_controller();

#endif


#ifndef PWM_H
#define PWM_H

#include "main.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "motor.h"

#define PWM_FREQ 50
#define PWM_PERIOD_MS 1000*(1/PWM_FREQ)

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

void pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, float d);
void pwm_initialization();

#endif
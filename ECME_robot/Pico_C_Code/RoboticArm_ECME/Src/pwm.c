#include "pwm.h"
#include "pico/stdlib.h"
#include "motor.h"
#include "main.h"

uint chan_motors[NUMBER_OF_MOTORS];
uint slice_motors[NUMBER_OF_MOTORS];
struct pwmSignal duty_cycle_motors;

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
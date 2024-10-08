#ifndef MOTOR_H
#define MOTOR_H

#define MOTOR_1 0
#define MOTOR_2 1
#define MOTOR_3 2
#define MOTOR_4 3
#define MOTOR_6 4
#define NUMBER_OF_MOTORS 5
#define PIN_MOTOR_1 21
#define PIN_MOTOR_2 20
#define PIN_MOTOR_3 19
#define PIN_MOTOR_4 18
#define PIN_MOTOR_6 17

#include "pico/stdlib.h"

void motor_move(uint slice_motors[], uint chan_motors[]);
void duty_cycle_set(float theta1, float theta2, float theta3, float theta4, uint config);

#endif
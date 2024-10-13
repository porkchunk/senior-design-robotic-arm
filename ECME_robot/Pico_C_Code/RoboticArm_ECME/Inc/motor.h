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
#define PIN_MOTOR_5 17
#define PIN_MOTOR_6 16
#define PIN_MOTOR_7 15

#include "pico/stdlib.h"

void motor_move();
void duty_cycle_set(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6, uint config);

#endif
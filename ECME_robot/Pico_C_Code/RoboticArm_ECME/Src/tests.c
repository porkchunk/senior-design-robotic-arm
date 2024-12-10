#include "tests.h"
#include "motor.h"
#include "robot_commands.h"
#include "robot_modes.h"
#include "interrupts.h"
#include "adc.h"
#include "SPI.h"

void test_motor_move(int motor){
    if(motor == 2){
        duty_cycle_motors.motor2 = (1.65/20);
        pwm_set_freq_duty(slice_motors[MOTOR_2], chan_motors[MOTOR_2], PWM_FREQ, duty_cycle_motors.motor2);
        pwm_set_enabled(slice_motors[MOTOR_2], true);
        sleep_ms(200);

        duty_cycle_motors.motor2 = 0;
        pwm_set_freq_duty(slice_motors[MOTOR_2], chan_motors[MOTOR_2], PWM_FREQ, duty_cycle_motors.motor2);
        pwm_set_enabled(slice_motors[MOTOR_2], true);
        sleep_ms(2000);
    }
    if(motor == 4){
        duty_cycle_motors.motor4 = 0.45;
        pwm_set_freq_duty(slice_motors[MOTOR_4], chan_motors[MOTOR_4], PWM_FREQ, duty_cycle_motors.motor4);
        pwm_set_enabled(slice_motors[MOTOR_4], true);
        sleep_ms(200);

        duty_cycle_motors.motor4 = 0;
        pwm_set_freq_duty(slice_motors[MOTOR_4], chan_motors[MOTOR_4], PWM_FREQ, duty_cycle_motors.motor4);
        pwm_set_enabled(slice_motors[MOTOR_4], true);
        sleep_ms(2000);
    }

    if(motor == 5){
        gpio_put(DIRECTION_MOTOR_5, false);
        duty_cycle_motors.motor5 = 0.05;
        pwm_set_freq_duty(slice_motors[MOTOR_5], chan_motors[MOTOR_5], PWM_FREQ, duty_cycle_motors.motor5);
        pwm_set_enabled(slice_motors[MOTOR_5], true);
        sleep_ms(100);

        duty_cycle_motors.motor5 = 0;
        pwm_set_freq_duty(slice_motors[MOTOR_5], chan_motors[MOTOR_5], PWM_FREQ, duty_cycle_motors.motor4);
        pwm_set_enabled(slice_motors[MOTOR_5], true);
        sleep_ms(2000);
    }
}

void test_claw_move(){
        claw_position = true;
        claw_move();
        sleep_ms(1000);
        claw_position = false;
        claw_move();
        sleep_ms(1000);
}

void PID_one_motor(){
    //target_position[MOTOR_5] = 2410;
    target_position[MOTOR_5] = 23;
    sleep_ms(10000);
    target_position[MOTOR_5] = 2400;
    sleep_ms(10000);
    //printf("actual_position: %d\n", actual_position[MOTOR_4]);
    //sleep_ms(9000);
    //target_position[MOTOR_5] = 2410;
    //sleep_ms(9000);
    //printf("Actual position:%d\n", actual_position[MOTOR_1]);
    //calculate_PID();
    //motor_move();
}


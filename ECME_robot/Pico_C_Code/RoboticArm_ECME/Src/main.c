#include <math.h>
#include <stdio.h>

#include "matrix.h"
#include "robot_commands.h"
#include "motor.h"
#include "interrupts.h"
#include "adc.h"
#include "robot_modes.h"
#include "timer.h"
#include "main.h"
#include "SPI.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/multicore.h"

void main_core1(){

}


void main()
{
    //Sets system clock frequency
    set_sys_clock_khz(SYS_FREQ_KHZ, true);

    //All initializations 
    stdio_init_all();
    motor_initialization();
    adc_initialization();
    interrupt_initialization();
    initialize_auto_manual_pin();
    initialize_start_auto_mode_pin();
    SPI_initialization();

    //Positions robot in initial position and lets pico know where robot is
    set_zero_position();
    set_initial_position();

    uint64_t start;
    uint64_t end;

    float matrix1[5][6] = {{-1,2,-3,4,5,3},{-1,2,3,4,-5,3},{1,2,3,4,5,3},{1,2,3,4,5,3},{1,2,3,4,5,3}};
    float matrix2[5][6] = {0};
    float matrix_result[6][5];

    float jacobian[5][6];
    float jacobian_inverse[6][5];

    float value;

    while (true) {
        calculate_PID();
        forward_kinematics(0,M_PI/2,-M_PI/2,0,0,0,position);
        printf("X: %f \n", position[0]);
        printf("Y: %f \n", position[1]);
        printf("Z: %f \n", position[2]);
        printf("PITCH: %f \n", position[3]);
        printf("YAW: %f \n\n", position[4]);

        jacobian_function(0,M_PI/2,-M_PI/2,0,0,0,jacobian);
        display(5,6,jacobian);
        /*
        position[0] = STARTING_X - 3;
        position[1] = STARTING_Y + 4;
        position[2] = STARTING_Z + 3;
        position[3] = 0;
        position[4] = 0;

        start = time_us_64();
        robot_move(5, position);
        end = time_us_64();
        printf("time_to_end: %llu \n", end - start);

        forward_kinematics(theta[0], theta[1], theta[2], theta[3], theta[4], theta[5], position);
        
        printf("Xe: %f \n", position[0]);
        printf("Ye: %f \n", position[1]);
        printf("Ze: %f \n", position[2]);
        printf("PITCHe: %f \n", position[3]);
        printf("YAWe: %f \n\n", position[4]);
        
        position[0] = STARTING_X;
        position[1] = STARTING_Y;
        position[2] = STARTING_Z;
        position[3] = 0;
        position[4] = 0;

        start = time_us_64();
        robot_move(5, position);
        end = time_us_64();
        printf("time_to_initial: %llu \n", end - start);

        forward_kinematics(theta[0], theta[1], theta[2], theta[3], theta[4], theta[5], position);
        printf("Xi: %f \n", position[0]);
        printf("Yi: %f \n", position[1]);
        printf("Zi: %f \n", position[2]);
        printf("PITCHi: %f \n", position[3]);
        printf("YAWi: %f \n\n", position[4]);
        */
        /*
        jacobian_function(0,M_PI/2,-M_PI/2,0,0,0,jacobian);
        start = time_us_64();
        inverse(jacobian, jacobian_inverse);
        end = time_us_64();
        
        printf("X: %0.2f \n", position[0]);
        printf("Y: %0.2f \n", position[1]);
        printf("Z: %0.2f \n", position[2]);        
        printf("ROLL: %0.2f \n", position[3]);
        printf("PITCH: %0.2f \n", position[4]);
        printf("YAW: %0.2f \n", position[5]);
        
        printf("time: %llu \n", end - start);
        */
        sleep_ms(1000);
        /*
        if(gpio_get(AUTO_MANUAL_SWITCH_PIN) == true){
            manual_mode();
        }
        else{
            automatic_mode();
        }
        */
    }
}

#include <math.h>
#include <stdio.h>

#include "matrix.h"
#include "robot_commands.h"
#include "motor.h"
#include "pwm.h"
#include "interrupts.h"
#include "adc.h"
#include "robot_modes.h"
#include "timer.h"
#include "main.h"
#include "I2C.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/multicore.h"


int main()
{
    //Sets system clock frequency
    set_sys_clock_khz(SYS_FREQ_KHZ, true);

    //All initializations 
    stdio_init_all();
    pwm_initialization();
    adc_initialization();
    I2C_initialization();
    interrupt_initialization();
    initialize_auto_manual_pin();
    initialize_start_auto_mode_pin();

    //Positions robot in initial position and lets pico know where robot is
    set_zero_position();

    uint64_t start;
    uint64_t end;

    float matrix1[5][6] = {{1,2,3,4,5,3},{1,2,3,4,5,3},{1,2,3,4,5,3},{1,2,3,4,5,3},{1,2,3,4,5,3}};
    float matrix2[5][6] = {0};
    float matrix_result[6][5];

    float jacobian[5][6];
    float jacobian_inverse[6][5];

    float theta1 = M_PI;
    float theta2 = M_PI/2;
    float theta3 = -M_PI/2;
    float theta4 = -M_PI;
    float theta5 = M_PI/2;
    float theta6 = 0;

    xyzpitch[0] = STARTING_X + 0.03;
    xyzpitch[1] = STARTING_Y + 0.03;
    xyzpitch[2] = STARTING_X + 0.03;
    xyzpitch[3] = 0;
    xyzpitch[4] = 0;
    
    while (true) {

        xyzpitch[0] = STARTING_X + 0.03 + xyzpitch[0];
        xyzpitch[1] = STARTING_Y + 0.03 + xyzpitch[1];
        xyzpitch[2] = STARTING_X + 0.03 + xyzpitch[2];
        xyzpitch[3] = 0;
        xyzpitch[4] = 0;

        jacobian_function(0,M_PI/2,-M_PI/2,0,0,0,jacobian);

        start = time_us_64();
        pseudo_inverse(jacobian, jacobian_inverse);
        end = time_us_64();

        display(6,5,jacobian_inverse);

        printf("%llu \n", end - start);
        //robot_move(xyzpitch);

        /*
        xyzpitch[0] = STARTING_X;
        xyzpitch[1] = STARTING_Y;
        xyzpitch[2] = STARTING_Z;
        xyzpitch[3] = STARTING_PITCH;
        xyzpitch[4] = STARTING_YAW;

        start = time_us_64();
        robot_move(xyzpitch);
        end = time_us_64();
        printf("%llu \n", end - start);
        /*
        jacobian_function(0,M_PI/2,-M_PI/2,0,0,0,jacobian);
        start = time_us_64();
        inverse(jacobian, jacobian_inverse);
        end = time_us_64();
        
        printf("X: %0.2f \n", xyzpitch[0]);
        printf("Y: %0.2f \n", xyzpitch[1]);
        printf("Z: %0.2f \n", xyzpitch[2]);        
        printf("ROLL: %0.2f \n", xyzpitch[3]);
        printf("PITCH: %0.2f \n", xyzpitch[4]);
        printf("YAW: %0.2f \n", xyzpitch[5]);
        
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

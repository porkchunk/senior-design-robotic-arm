#include <math.h>

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
    set_initial_position();

    uint64_t start;
    uint64_t end;

    float jacobian[4][4];
    float jacobian_inverse[4][4];
    
    while (true) {
        /*
        xyzpitch[0] = 30;
        xyzpitch[1] = STARTING_Y;
        xyzpitch[2] = 9;
        xyzpitch[3] = 0;

        start = time_us_64();
        robot_move(xyzpitch);
        end = time_us_64();

        float theta1 = map_function(20*duty_cycle_motors.motor1, 0.5, 2.5, -M_PI/2, M_PI/2);
        float theta2 = map_function(20*duty_cycle_motors.motor2, 0.5, 2.5, M_PI, 0);
        float theta3 = map_function(20*duty_cycle_motors.motor3, 0.56, 2.06, (-3*M_PI)/4, 0);
        float theta4 = map_function(20*duty_cycle_motors.motor4, 0.5, 2.5, -M_PI/2, M_PI/2);

        printf("Theta1: %0.4f\n", theta1);
        printf("Theta2: %0.4f\n", theta2);
        printf("Theta3: %0.4f\n", theta3);
        printf("Theta4: %0.4f\n", theta4);

        forward_kinematics(theta1, theta2, theta3, theta4, xyzpitch);

        printf("X: %0.4f \n", xyzpitch[0]);
        printf("Y: %0.4f \n", xyzpitch[1]);
        printf("Z: %0.4f \n", xyzpitch[2]);
        printf("PITCH: %0.4f \n\n", xyzpitch[3]);

        sleep_ms(500);

        printf("%llu \n", end - start);
        
        xyzpitch[0] = STARTING_X;
        xyzpitch[1] = STARTING_Y;
        xyzpitch[2] = STARTING_Z;
        xyzpitch[3] = 0;

        robot_move(xyzpitch);

        sleep_ms(500);
        */
        
        if(gpio_get(AUTO_MANUAL_SWITCH_PIN) == true){
            manual_mode();
        }
        else{
            automatic_mode();
        }
    
    }
}

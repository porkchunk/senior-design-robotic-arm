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

    uint64_t start;
    uint64_t end;

    float jacobian[4][4];
    float jacobian_inverse[4][4];
    
    while (true) {

        xyzpitch[0] = STARTING_X + 0.03;
        xyzpitch[1] = STARTING_Y + 0.03;
        xyzpitch[2] = STARTING_X + 0.03;
        xyzpitch[3] = 0;

        robot_move(xyzpitch);
        
        xyzpitch[0] = STARTING_X;
        xyzpitch[1] = STARTING_Y;
        xyzpitch[2] = STARTING_X;
        xyzpitch[3] = 0;

        

        sleep_ms(500);
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

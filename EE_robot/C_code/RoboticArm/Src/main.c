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

#include "pico/stdlib.h"
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
    interrupt_initialization();
    initialize_auto_manual_pin();
    initialize_start_auto_mode_pin();

    //Positions robot in initial position and lets pico know where robot is
    set_zero_position();
    set_initial_position();

    while (true) {
        automatic_mode();
    }
    /*
        if(gpio_get(AUTO_MANUAL_SWITCH_PIN) == true){
            automatic_mode();
        }
        else{
            manual_mode();
        }
    */
}

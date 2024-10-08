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
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

int main()
{
    set_sys_clock_khz(SYS_FREQ_KHZ, true);

    stdio_init_all();
    pwm_initialization();
    adc_initialization();
    I2C_initialization();
    interrupt_initialization();

    set_initial_position();

    

    uint64_t start;
    uint64_t end;
    uint64_t time_taken;
    
    while (true) {
        manual_mode();
    }
}

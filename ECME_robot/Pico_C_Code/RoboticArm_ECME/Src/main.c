#include <math.h>
#include <stdio.h>

#include "matrix.h"
#include "robot_commands.h"
#include "motor.h"
#include "interrupts.h"
#include "adc.h"
#include "robot_modes.h"
#include "main.h"
#include "SPI.h"
#include "tests.h"
#include "UART.h"

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
    motor_initialization();
    adc_initialization();
    interrupt_initialization();
    initialize_auto_manual_pin();
    initialize_start_auto_mode_pin();
    SPI_initialization();
    pwm_initialization();
    UART_initialization();
    button_initialization();

    //Positions robot in initial position
    set_zero_position();
    //int number;
    //multicore_launch_core1(main_core1);
    while (true){
        move_motor_with_controller();
    }
}

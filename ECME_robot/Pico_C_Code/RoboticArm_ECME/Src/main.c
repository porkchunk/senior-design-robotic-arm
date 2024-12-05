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

void main_core1(){
    while(false){ //In future should only enable this when certain conditions are met
        read_encoders();
        calculate_PID();
        motor_move();
    }
}

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

    //Positions robot in initial position
    set_zero_position();

    multicore_launch_core1(main_core1);
    float number1;
    float number2;
    float number3;
    float number4;
    float number5;
    float number6;
    while (true){
        number1 = MCP3208(0);
        number2 = MCP3208(1);
        number3 = MCP3208(2);
        number4 = MCP3208(3);
        number5 = MCP3208(4);
        number6 = MCP3208(5);

    /*
        printf("Channel0: %0.2f\n", (3.3/4095.0)*number1);
        printf("Channel1: %0.2f\n", (3.3/4095.0)*number2);
    */
        printf("Channel4: %0.2f\n", (3.3/4095.0)*number5);  
        printf("Channel4: %0.2f\n\n", (3.3/4095.0)*number6); 
    /*
        printf("Channel2: %0.2f\n", (3.3/4095.0)*number3);
        printf("Channel3: %0.2f\n", (3.3/4095.0)*number4);

    */

        sleep_ms(1000);
        /*
        number = send_and_receive_UART();
        if(number != -1){
            printf("%d\n", number);
        }
        */
    }
}

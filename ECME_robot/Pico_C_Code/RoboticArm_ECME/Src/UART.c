#include "UART.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"

void UART_initialization(){
    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));

    uart_init(uart0, 115200);

    
}
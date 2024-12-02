#include "UART.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <stdlib.h>

void UART_initialization(){
    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));

    uart_init(uart0, 115200);
}

int send_and_receive_UART(){
    int index = -1;
    uart_putc_raw(uart0,'s');  
    sleep_ms(2000); 
    
    if (uart_is_readable(uart0)) {
        char message[256];  
        int len = 0;
        while (uart_is_readable(uart0)) {
            char c = uart_getc(uart0); 
            message[len++] = c;
            if (c == '\n' || len == sizeof(message) - 1) {  
                break;
            }
        }
        message[len] = '\0'; 
        //printf("Received: %s\n", message);  // Print the received message
        index = atoi(message);
    }
     
    return index;
}
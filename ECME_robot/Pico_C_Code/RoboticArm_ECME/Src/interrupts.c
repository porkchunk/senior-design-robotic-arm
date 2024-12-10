#include "interrupts.h"
#include "robot_commands.h"
#include "motor.h"
#include "robot_modes.h"
#include <stdio.h>

uint64_t last_interrupt_time1 = 0;
uint64_t last_interrupt_time2 = 0;

void gpio_callback(uint gpio, uint32_t events) {
    uint64_t interrupt_time = time_us_64()/1e6;
    if(interrupt_time - last_interrupt_time1 > 0.2){
        motor_number = ((motor_number + 1) % 6);
        if(motor_number == 2){
            motor_number = 3;
        }
        last_interrupt_time1 = interrupt_time;
    }
}

//Initialize interrupts
void interrupt_initialization(){
    gpio_set_irq_enabled_with_callback(BUTTON_LEFT, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

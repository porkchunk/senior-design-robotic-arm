#include "interrupts.h"
#include "robot_commands.h"\

uint64_t last_interrupt_time = 0;

void gpio_callback(uint gpio, uint32_t events) {
    uint64_t interrupt_time = time_us_64()/1e6;
    if(interrupt_time - last_interrupt_time > 0.2){
        claw_position = !claw_position;
    }
    last_interrupt_time = interrupt_time;
}

//Initialize interrupts
void interrupt_initialization(){
    gpio_set_irq_enabled_with_callback(1, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}
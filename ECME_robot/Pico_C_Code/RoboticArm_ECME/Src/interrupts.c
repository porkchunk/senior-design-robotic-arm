#include "interrupts.h"
#include "robot_commands.h"

void gpio_callback(uint gpio, uint32_t events) {
    claw_position = !claw_position;
}

//Initialize interrupts
void interrupt_initialization(){
    gpio_set_irq_enabled_with_callback(1, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}
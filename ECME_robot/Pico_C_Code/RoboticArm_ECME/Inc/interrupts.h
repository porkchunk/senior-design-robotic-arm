#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#include "hardware/gpio.h"


void interrupt_initialization();
void gpio_callback(uint gpio, uint32_t events);

#endif
#include "adc.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"

const float conversion_factor = 3.3f / (1 << 12);

void adc_initialization(){
    adc_init();
    adc_gpio_init(28);
    adc_gpio_init(27);
    adc_gpio_init(26);
}
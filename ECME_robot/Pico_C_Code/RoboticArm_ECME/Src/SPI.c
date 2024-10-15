#include "SPI.h"
#include "motor.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"


static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

void SPI_initialization() {
    spi_init( SPI_PORT, 1350000 );
    spi_set_format( SPI_PORT, 8, 0, 0, SPI_MSB_FIRST );

    gpio_set_function( PIN_CLK, GPIO_FUNC_SPI );
    gpio_set_function( PIN_TX, GPIO_FUNC_SPI );
    gpio_set_function( PIN_RX, GPIO_FUNC_SPI );
    gpio_set_function( PIN_CS, GPIO_FUNC_SPI );

    gpio_init( PIN_CS );
    gpio_set_dir( PIN_CS, GPIO_OUT );
    gpio_put( PIN_CS, 1 );
}

float MCP3008(int chan) {
    cs_select();

    size_t len = 3;
    uint8_t buffer[3];
    buffer[0] = 1;
    buffer[1] = (0b1000 + chan) << 4;
    buffer[2] = 0;

    uint8_t returnData[3];
    
    spi_write_read_blocking( SPI_PORT, buffer, returnData, sizeof(buffer));

    int data = ((returnData[1] & 3) << 8) | returnData[2];

    float val = (3.3 / 1023.0 ) * data;

    cs_deselect();

    return val;
}

void read_encoders(){
    for(int i=0; i<6; ++i){
        actual_position[i] = MCP3008(i);
    }
}


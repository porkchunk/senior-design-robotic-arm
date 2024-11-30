#ifndef SPI_H
#define SPI_H

#include "hardware/spi.h"
#include "pico/stdlib.h"

#define PIN_CS 1
#define PIN_TX 3
#define PIN_RX 0
#define PIN_CLK 2

#define SPI_PORT spi0

static inline void cs_select();
static inline void cs_deselect();
void SPI_initialization();
float MCP3208(int chan);
void read_encoders();

#endif
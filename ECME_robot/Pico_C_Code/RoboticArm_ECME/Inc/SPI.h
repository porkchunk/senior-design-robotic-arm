#ifndef SPI_H
#define SPI_H

#include "hardware/spi.h"
#include "pico/stdlib.h"

#define PIN_CS 13
#define PIN_TX 11
#define PIN_RX 12
#define PIN_CLK 10

#define SPI_PORT spi1

static inline void cs_select();
static inline void cs_deselect();
void SPI_initialization();
int MCP3208(int chan);
void read_encoders();

#endif
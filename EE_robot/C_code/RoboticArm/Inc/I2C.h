#ifndef I2C_H
#define I2C_H

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define I2C_FREQ 400*100

void I2C_initalization();

#endif
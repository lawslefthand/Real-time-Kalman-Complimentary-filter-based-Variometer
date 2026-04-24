/*
 * bmp280_secondary.h
 *
 *  Created on: Apr 4, 2026
 *      Author: danba
 */

#ifndef INC_BMP280_SECONDARY_H_
#define INC_BMP280_SECONDARY_H_

#define BMP280_I2C_ADDR (0x76 << 1)


#include "stm32f4xx.h"
#include <math.h>

void bmp_hal_i2c_write(uint8_t reg_addr, uint8_t value);
uint8_t bmp_hal_i2c_read(uint8_t reg_addr);
double temperature_secondary(int x);
double pressure_secondary(void);
void bmp_i2c_setup_secondary(void);
float altitude_calc_secondary(void);

#endif /* INC_BMP280_SECONDARY_H_ */

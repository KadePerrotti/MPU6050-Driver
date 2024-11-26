/*
 * utils.h
 *
 *  Created on: Nov 25, 2024
 *      Author: Kade
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>
#include "i2c.h"

#include "REG_ADDRESSES.h"

#define SIZE_1_BYTE (1)

#define HAL_I2C_TIMEOUT (100)

uint16_t init_mpu6050(void);

#endif /* INC_UTILS_H_ */

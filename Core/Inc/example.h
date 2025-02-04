/*
 * example.h
 *
 *  Created on: Feb 4, 2025
 *      Author: Kade
 */

#ifndef INC_EXAMPLE_H_
#define INC_EXAMPLE_H_

/**
 * Wrapper around HAL_I2C_Mem_Write. 
 * TODO: Convert to Macro, include return
 */
int MPU6050_REG_WRITE_STM32(uint16_t regAddr, uint8_t regValue);

/**
 * Wrapper around HAL_I2C_Mem_Read
 * //todo add return
 */
int MPU6050_REG_READ_STM32(uint16_t regAddr, uint8_t* valAddr);

int MPU6050_BURST_READ_STM32(uint16_t regAddr, uint8_t* data, uint16_t bytes);

#endif /* INC_EXAMPLE_H_ */

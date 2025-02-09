/*
 * example.c
 *
 *  Created on: Feb 4, 2025
 *      Author: Kade
 */

#include "i2c.h"

#include "MPU6050.h"
#include "REG_OPTIONS.h"

int MPU6050_REG_WRITE_STM32(uint16_t regAddr, uint8_t regValue)
{
    //todo: return the hal status
    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU_6050_HAL_I2C_ADDR,
        regAddr,
        SIZE_1_BYTE,
        &regValue,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
    return 0;
}

int MPU6050_REG_READ_STM32(uint16_t regAddr, uint8_t* valAddr)
{
    //todo: return the hal status
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        regAddr,
        SIZE_1_BYTE,
        valAddr,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
    return 0;
}

int MPU6050_BURST_READ_STM32(uint16_t regAddr, uint8_t* data, uint16_t bytes)
{
     //todo: return the hal status
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        regAddr,
        SIZE_1_BYTE,
        data,
        bytes,
        HAL_I2C_TIMEOUT
    );
    return 0;
}
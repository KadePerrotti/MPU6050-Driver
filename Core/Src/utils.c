/*
 * utils.c
 *
 *  Created on: Nov 25, 2024
 *      Author: Kade
 */
#include "utils.h"

uint16_t init_mpu6050(void)
{
    uint8_t who_am_i_res = 0; //result stored here
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR, 
        REG_WHO_AM_I, 
        SIZE_1_BYTE, 
        &who_am_i_res, 
        SIZE_1_BYTE, 
        HAL_I2C_TIMEOUT);
    if(status != HAL_OK)
    {
        return status;
    }
    if(who_am_i_res != MPU_6050_ADDR)
    {
        return -1;
    }
    return 0;
}


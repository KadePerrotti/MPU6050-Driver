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
    uint8_t* writeBuff;

    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR, 
        REG_WHO_AM_I, 
        SIZE_1_BYTE, 
        &who_am_i_res, 
        SIZE_1_BYTE, 
        HAL_I2C_TIMEOUT
    );
    if(status != HAL_OK)
    {
        return status;
    }
    if(who_am_i_res != MPU_6050_ADDR)
    {
        return -1;
    }

    //reset the power managment register
    writeBuff[0] = PWR_MGMT_DEV_RESET | PWR_MGMT_CLK_SEL_INTERNAL;
    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU_6050_HAL_I2C_ADDR,
        REG_PWR_MGMT_1,
        SIZE_1_BYTE,
        writeBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
    
    //setup the config register for max filtering
    writeBuff[0] = DLPF_MAX_FILTER | EXT_SYNC_OFF;
    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU_6050_HAL_I2C_ADDR,
        REG_CONFIG,
        SIZE_1_BYTE,
        writeBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    //setup the accelerometer
    writeBuff[0] = ACCEL_FS_2G;
    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU_6050_HAL_I2C_ADDR,
        REG_ACCEL_CONFIG,
        SIZE_1_BYTE,
        writeBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    //setup the gyro sample rate
    writeBuff[0] = SAMPLE_RATE_100Hz;
    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU_6050_HAL_I2C_ADDR,
        REG_SMPRT_DIV,
        SIZE_1_BYTE,
        writeBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    //setup the gyro
    writeBuff[0] = GYRO_FS_SEL_250_DPS;
    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU_6050_HAL_I2C_ADDR,
        REG_GYRO_CONFIG,
        SIZE_1_BYTE,
        writeBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
    return 0;
}


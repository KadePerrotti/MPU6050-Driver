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

void read_setup_registers(void)
{
    uint8_t readBuff = 0;
    char txBuff[100];
    int uart_buf_len = 0;
    
    uart_buf_len = sprintf(txBuff, "\r\n-------------------------\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, uart_buf_len, 100);
    txBuff[0] = '\0';

    //config register
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_CONFIG,
        SIZE_1_BYTE,
        &readBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
    uint8_t fsync_val = readBuff & 0x38;
    uint8_t dlpf_val = readBuff & 0x7;
    uart_buf_len = sprintf(
        txBuff, 
        "\r\n\n Config Reg: \r\n  FSYNC: %d, %c\r\n  DLPF: %d, %c\r\n", 
        fsync_val, 
        fsync_val == EXT_SYNC_OFF ? 't' : 'f',
        dlpf_val,
        dlpf_val == DLPF_MAX_FILTER ? 't' : 'f'
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, uart_buf_len, 100);
    txBuff[0] = '\0';
    readBuff = 0;

    //accelerometer config register
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_ACCEL_CONFIG,
        SIZE_1_BYTE,
        &readBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
    uint8_t fs_val = readBuff & 0x18;
    uart_buf_len = sprintf(
        txBuff, 
        "\r\n\n Accel Config Reg: \r\n  Full Scale: %d, %c\r\n", 
        fs_val, 
        fs_val == ACCEL_FS_2G ? 't' : 'f'
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, uart_buf_len, 100);
    txBuff[0] = '\0';
    readBuff = 0;

    //gyro sample rate divider
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_SMPRT_DIV,
        SIZE_1_BYTE,
        &readBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
    uint8_t rate_div = readBuff & 0xFF;
    uart_buf_len = sprintf(
        txBuff, 
        "\r\n\n Gyro SR Div: \r\n  Divider: %d, %c\r\n", 
        rate_div, 
        rate_div == SAMPLE_RATE_100Hz ? 't' : 'f'
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, uart_buf_len, 100);
    txBuff[0] = '\0';
    readBuff = 0;

    //gyro config
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_GYRO_CONFIG,
        SIZE_1_BYTE,
        &readBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
    uint8_t gyro_sel = readBuff & 0x18;
    uart_buf_len = sprintf(
        txBuff, 
        "\r\n\n Gyro Full Scale: \r\n  FS: %d, %c\r\n", 
        gyro_sel, 
        gyro_sel == GYRO_FS_SEL_250_DPS ? 't' : 'f'
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, uart_buf_len, 100);
    txBuff[0] = '\0';
    readBuff = 0;


}


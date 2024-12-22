/*
 * utils.c
 *
 *  Created on: Nov 25, 2024
 *      Author: Kade
 */
#include "utils.h"

void MPU6050_REG_WRITE(uint16_t regAddr, uint8_t regValue)
{
    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU_6050_HAL_I2C_ADDR,
        regAddr,
        SIZE_1_BYTE,
        &regValue,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
}

uint16_t init_mpu6050(void)
{
    char txBuff[100];
    int uart_buf_len = 0;

    uart_buf_len = sprintf(txBuff, "\r\n-------------------------\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, uart_buf_len, 100);
    txBuff[0] = '\0';

    //reset the power managment register
    MPU6050_REG_WRITE(REG_PWR_MGMT_1, PWR_MGMT_DEV_RESET);
    HAL_Delay(100);

    //Reset fifo, i2c master, sensor signal paths and sensors
    //via user ctrl register
    MPU6050_REG_WRITE(REG_USER_CTRL, FIFO_RESET | I2C_MST_RESET | SIG_COND_RESET);
    HAL_Delay(100);

    //set clock source
    MPU6050_REG_WRITE(REG_PWR_MGMT_1, PWR_MGMT_CLK_SEL_INTERNAL);

    //setup 2nd power management register
    MPU6050_REG_WRITE(REG_PWR_MGMT_2, STBY_YA | STBY_ZA | STBY_YG | STBY_ZG); //only leave gyro and accel x axis awake
    
    //setup the config register for max filtering
    MPU6050_REG_WRITE(REG_CONFIG, DLPF_MAX_FILTER | EXT_SYNC_OFF);

    //setup the gyro
    MPU6050_REG_WRITE(REG_GYRO_CONFIG, GYRO_FS_SEL_500_DPS);   
    
    //setup the accelerometer
    MPU6050_REG_WRITE(REG_ACCEL_CONFIG, ACCEL_FS_4G);

    //setup the sample rate divider
    MPU6050_REG_WRITE(REG_SMPRT_DIV, SAMPLE_RATE_100Hz);
    
    HAL_Delay(1000);

    return 0;
}

void read_setup_registers(void)
{
    uint8_t readBuff = 0;
    char txBuff[100];
    int uart_buf_len = 0;

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
        gyro_sel == GYRO_FS_SEL_500_DPS ? 't' : 'f'
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
        fs_val == ACCEL_FS_4G ? 't' : 'f'
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, uart_buf_len, 100);
    txBuff[0] = '\0';
    readBuff = 0;

    //sample rate divider
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


}


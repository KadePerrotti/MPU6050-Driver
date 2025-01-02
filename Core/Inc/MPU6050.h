/*
 * utils.h
 *
 *  Created on: Nov 25, 2024
 *      Author: Kade
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <REG_OPTIONS.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "i2c.h"
#include "usart.h"



#define SIZE_1_BYTE (1)

#define HAL_I2C_TIMEOUT (100)

#define S_TO_MS(s) (s * 1000)

/**
 * Sets configuration registers to default values
 * 1. Reset Power Management Register - 0x6B
 * 2. Reset fifo, i2c master, sensor signal paths, and sensors
 * using User Control register - 0x6A
 * 3. Sets clock source to internal via Power Management Register - 0x6B
 * 4. Awakens all gyro and accel axes via Power Management 2 Register - 0x6C
 *    (turn off unused axes later)
 */
uint16_t init_mpu6050(void);

/**
 * Self-test function that reads back configuration registers, 
 * and confirms if the expected values were written.
 * Checks: REG_CONFIG, REG_GYRO_CONFIG, REG_ACCEL_CONFIG,
 * REG_SMPRT_DIV
 */
void read_setup_registers(void);

/**
 * Wrapper around HAL_I2C_Mem_Write. 
 * TODO: Convert to Macro, include return
 */
void MPU6050_REG_WRITE(uint16_t regAddr, uint8_t regValue);

/**
 * Accelerometer readings are 2 bytes, stored in two registers on the
 * MPU6050. Reads both, then combines and scales value.
 * @param address: most significant accel axis register
 * @param scaler: scale the raw measurement to the expected unit
 * @return The accel measurement in G (9.8m/s^2)
 */
float read_accel_axis(uint8_t address, uint16_t scaler);

/**
 * Gyroscope readings are 2 bytes, stored in two registers on the
 * MPU6050. Reads both, then combines and scales value.
 * @param address most significant gyro axis register
 * @param scaler: scale the raw measurement to the expected unit
 * @return The gyro measurement in DPS (degrees per second)
 */
float read_gyro_axis(uint8_t address, uint16_t scaler);
#endif /* INC_MPU6050_H_ */

/*
 * testFunctions.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Kade
 */

#ifndef INC_TESTFUNCTIONS_H_
#define INC_TESTFUNCTIONS_H_

#include "MPU6050.h"

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

/**
 * Self-test function that reads back configuration registers, 
 * and confirms if the expected values were written.
 * Checks: REG_CONFIG, REG_GYRO_CONFIG, REG_ACCEL_CONFIG,
 * REG_SMPRT_DIV
 */
void read_setup_registers(MPU6050_REG_READ_TYPE readReg);

/**
 * testing function that reads each gyro and accel axis individually
 * from the individual register, then does a debug print
 */
void poll_axes_individually(MPU6050_REG_READ_TYPE readReg);

/**
 * Periodically checks the fifo count. Uses readPeriod, sampleRate, and
 * numAxes to determine if the count matches the expected count.
 * @param readPeriodMs: How often to check the fifo count
 * @param sampleRate: rate at which an axis is written to the fifo
 * @param numAxes: Number of axis being written to the fifo (gyro and accel each have 3)
 */
void fifo_count_test(uint16_t readPeriodMs, uint16_t sampleRate, uint8_t numAxes, MPU6050_BURST_READ_TYPE burstRead, MPU6050_REG_READ_TYPE readReg);

/**
 * Periodically read the fifo. Convert raw data into gyro and accel readings
 * and print
 * Note: Expects the fifo to be collecting data from all 6 imu axes
 * @param readPeriodMs: How often to read the fifo
 */
void read_fifo_test(uint16_t readPeriodMs, MPU6050_BURST_READ_TYPE burstRead, MPU6050_REG_READ_TYPE readReg);

#endif /* INC_TESTFUNCTIONS_H_ */

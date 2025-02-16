/*
 * TEST_FUNCTIONS.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Kade Perrotti
 * Tests functions to ensure normal MPU6050 operation
 */

#ifndef INC_TESTFUNCTIONS_H_
#define INC_TESTFUNCTIONS_H_

#include "MPU6050.h"

typedef struct 
{
    //Contained in REG_CONFIG
    uint8_t fsync;
    uint8_t dlpf;

    //GYRO_CONFIG
    uint8_t gyro_sel;

    //ACCEL_CONFIG
    uint8_t fs;

    //SMPRT_DIV
    uint8_t rate_div;
} SETUP_REGISTERS;

/**
 * Self-test function that reads configuration registers
 * and masks the desired variables. Returns a struct containing the
 * them.
 * 
 * Checks: REG_CONFIG, REG_GYRO_CONFIG, REG_ACCEL_CONFIG,
 * REG_SMPRT_DIV
 */
SETUP_REGISTERS read_setup_registers(MPU6050_REG_READ_TYPE readReg);

/**
 * @brief testing function that reads each gyro and accel axis individually
 * from the individual register for the amount of time specified
 * @param sampleRate rate to sample each axis in Hz
 * @param sampleTime total amount of time to sample in ms
 * @param accelScaler scaler to transform raw accel data into g
 * @param gyroScaler scaler to transform raw gyro data into degrees per second
 * @param data array to place sample data into
 * @param readReg function that performs register reads on the MPU6050
 * @param delay function that blocks program execution for the specified ms
 * @param getTime function that returns the current tick time in ms
 */
void poll_axes_individually
(
    uint16_t sampleRate, 
    uint16_t sampleTime,
    float accelScaler,
    float gyroScaler, 
    float *data, 
    MPU6050_REG_READ_TYPE readReg,
    DELAY_MS_TYPE delay,
    TIME_MS_TYPE getTime
);

#endif /* INC_TESTFUNCTIONS_H_ */

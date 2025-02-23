/*
 * TEST_FUNCTIONS.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Kade Perrotti
 * Tests functions to ensure normal MPU6050 operation
 */

#ifndef INC_TESTFUNCTIONS_H_
#define INC_TESTFUNCTIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "MPU6050.h"

typedef struct 
{
    //Contained in REG_CONFIG
    uint8_t fsync;
    uint8_t dlpf;

    //GYRO_CONFIG
    uint8_t gyro_sel;

    //ACCEL_CONFIG
    uint8_t accel_sel;

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
 * @param readReg pointer to function that implements reading an mpu6050 register
 */
SETUP_REGISTERS read_setup_registers(MPU6050_REG_READ_TYPE readReg);

/**
 * Helper function that builds a string comparing expected configuration 
 * values to their actual.
 * @param expected The configuration values expected to have been read back
 * @param actual The actual configuration values read back
 * @param buff Points to where the string should be built
 * @return the size of the string built
 */
uint16_t build_setup_registers_string(SETUP_REGISTERS expected, SETUP_REGISTERS actual, char* buff);

/**
 * @brief testing function that reads each gyro and accel axis
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

/**
 * Builds a string containing results from poll_axes_individually
 * @param data array containing data from all 6 axis, in the order
 * accel x, y, z, gyro x, y, z
 * @param dataSize number of elements in data (not num bytes)
 * @param buff Points to where the string should be built
 * @return the size of the string built
 */
uint16_t build_poll_axes_string(float *data, uint16_t dataSize, char *buff);


/**
 * Helper function that builds a string representing the results
 *  of the accelerometer and gyroscope self test functions.
 * @param gyroResults struct containing results of gyro self test
 * @param accelResults struct containing results of accel self test
 * @param buff Points to where the string should be built
 * @return the size of the string built
 */
uint16_t build_self_tests_string(FACTORY_TEST_RESULTS gyroResults, FACTORY_TEST_RESULTS accelResults, char* buff);

#ifdef __cplusplus
}
#endif

#endif /* INC_TESTFUNCTIONS_H_ */

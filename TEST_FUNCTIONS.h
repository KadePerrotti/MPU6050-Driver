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

#define NUM_FIFO_COUNT_TESTS (10) //number of tests for fifo count test

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

/**
 * Periodically checks the fifo count. Uses readPeriod, sampleRate, and
 * numAxes to determine if the fifo will overflow. Will return
 * early if the number of bytes expected to fill the fifo each iteration exceeds
 * the fifo size. 
 * @param readPeriodMs How often to check the fifo count
 * @param sampleRate rate at which an axis is written to the fifo
 * @param numAxes Number of axis being written to the fifo (gyro and accel each have 3)
 * @param burstRead function that implements burst reads of MPU6050 registers
 * @param readReg function that implements single register reads of MPU6050 registers
 * @param delay function that blocks program execution for the specified ms
 * @param getTime function that returns the current tick time in ms
 * @param fifoCountResults array that save the fifo count each iteration
 * @param timePerIter array that saves the time it takes to read the fifo and count each iter
 * @param bytesPerRead number of bytes expected to fill fifo during each read
 */
void fifo_count_test(
    uint16_t readPeriodMs, 
    uint16_t sampleRate, 
    uint8_t numAxes, 
    MPU6050_BURST_READ_TYPE burstRead, 
    MPU6050_REG_READ_TYPE readReg,
    DELAY_MS_TYPE delay,
    TIME_MS_TYPE getTime,
    uint16_t *fifoCountResults, 
    uint32_t *timePerIter, 
    float *bytesPerRead
);

/**
 * Builds a string reporting results from fifo_count_test
 * @param fifoCountResults array of saved fifo counts
 * @param timePerIter time it took to read fifo and count for each iter
 * @param numBytesExpected the expected fifo count for each iteration
 * @param buff location of string
 */
uint16_t fifo_count_build_string(
    uint16_t *fifoCountResults, 
    uint32_t *timePerIter, 
    float bytesPerRead, 
    char *buff
);
#ifdef __cplusplus
}
#endif

#endif /* INC_TESTFUNCTIONS_H_ */

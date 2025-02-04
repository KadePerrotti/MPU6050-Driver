/*
 * utils.h
 *
 *  Created on: Nov 25, 2024
 *      Author: Kade
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>




#define SIZE_1_BYTE (1)

#define HAL_I2C_TIMEOUT (100)

#define S_TO_MS(s) (s * 1000)

#define BYTES_PER_MEASURE (2) //gyro, accel, temp measurements are 2 bytes each

#define FIFO_SIZE (1024) //size of fifo in bytes

//macro to turn raw data into a gyro or accel reading
#define TRANSFORM(upper, lower, scaler) \
    ((int16_t)(((uint16_t)upper << 8) | lower)) / (float)scaler;


typedef int MPU6050_REG_READ_TYPE(uint16_t, uint8_t*);
typedef int MPU6050_BURST_READ_TYPE(uint16_t, uint8_t*, uint16_t);
typedef int MPU6050_REG_WRITE_TYPE(uint16_t, uint8_t);

/**
 * Sets configuration registers to default values
 * 1. Reset Power Management Register - 0x6B
 * 2. Reset fifo, i2c master, sensor signal paths, and sensors
 * using User Control register - 0x6A
 * 3. Sets clock source to internal via Power Management Register - 0x6B
 * 4. Awakens all gyro and accel axes via Power Management 2 Register - 0x6C
 *    (turn off unused axes later)
 */
uint16_t init_mpu6050(MPU6050_REG_WRITE_TYPE writeReg);

/**
 * Accelerometer readings are 2 bytes, stored in two registers on the
 * MPU6050. Reads both, then combines and scales value.
 * @param address: most significant accel axis register
 * @param scaler: scale the raw measurement to the expected unit
 * @return The accel measurement in G (9.8m/s^2)
 */
float read_accel_axis(uint8_t address, uint16_t scaler, MPU6050_REG_READ_TYPE readReg);

/**
 * Gyroscope readings are 2 bytes, stored in two registers on the
 * MPU6050. Reads both, then combines and scales value.
 * @param address most significant gyro axis register
 * @param scaler: scale the raw measurement to the expected unit
 * @return The gyro measurement in DPS (degrees per second)
 */
float read_gyro_axis(uint8_t address, uint16_t scaler, MPU6050_REG_READ_TYPE readReg);

/**
 * Get the raw reading from any of the accelerometer or gyro axes
 * @param address: most significant axis register
 * @return the raw axis measurement. 
 */
int16_t read_raw_axis(uint8_t address, MPU6050_REG_READ_TYPE readReg);

typedef enum 
{
    FACTORY_TEST_PASS = 0,
    FACTORY_TEST_FAIL = -1
} FACTORY_TEST_RESULT;

/**
 * Runs a self test on the gyro. Steps:
 * 1. Set gyro's full scale range to 250dps
 * 2. Save gyro's output with self test disabled (TD)
 * 3. Enable self test register
 * 4. Save gyro's output with self test enabled (TE)
 * 5. SelfTestResponse (STR) = TE - TD
 * 6. Obtain Factory Trim values (FT)
 * 7. Use FT and STR to determine if each axis has passed
 */
FACTORY_TEST_RESULT gyro_self_test(MPU6050_REG_READ_TYPE readReg, MPU6050_REG_WRITE_TYPE writeReg);


/**
 * Runs a self test on the accelerometer. Steps:
 * 1. Set accelerometer's full scale range to 8g
 * 2. Save accel output with self test disabled (TD)
 * 3. Enable self test registers
 * 4. Save accel output with self test enabled (TE)
 * 5. SelfTestResponse (STR) = TE - TD
 * 6. Obtain Factory Trim values (FT)
 * 7. Use FT and STR to determine if each axis has passed
 */
FACTORY_TEST_RESULT accel_self_test(MPU6050_REG_READ_TYPE readReg, MPU6050_REG_WRITE_TYPE writeReg);


/**
 * Helper function to read the number of bytes currently in the fifo
 * Reads REG_FIFO_COUNT_H first, then REG_FIFO_COUNT_L, and concatenates
 * them
 */
uint16_t read_fifo_count(MPU6050_REG_READ_TYPE readReg);

#endif /* INC_MPU6050_H_ */

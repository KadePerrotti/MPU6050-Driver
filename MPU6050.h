/*
 * MPU6050.h
 *
 *  Created on: Nov 25, 2024
 *      Author: Kade Perrotti
 * Header file for working with the MPU6050. Contains typedefs, functions, 
 * and useful #defines for reading / writing to the MPU6050
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#define SIZE_1_BYTE (1)
#define S_TO_MS(s) (s * 1000)
#define MS_TO_S(ms) (ms / 1000.0f)
#define BYTES_PER_MEASURE (2) //gyro, accel, temp measurements are 2 bytes each
#define FIFO_SIZE (1024) //size of fifo in bytes

//if the change from factory trim is greater than abs(percent), the axis has failed
#define SELF_TEST_FAIL_PERCENT (14.0f)

//macro to turn raw data into a gyro or accel reading
#define TRANSFORM(upper, lower, scaler) \
    ((int16_t)(((uint16_t)upper << 8) | lower)) / (float)scaler;


/// @brief Functions in this library expect a parameter function
/// that implements register reads / writes for the user's specific
/// platform and protocol. See the readme and example repos for more information


/**
 * @brief Functions of MPU6050_REG_READ_TYPE should read a single byte
 * at the address specified by the first parameter into the memory
 * pointed to by the 2nd parameter.
 * @param 1: Address of the register to read
 * @param 2: Pointer to read the register's data into
 */
typedef int MPU6050_REG_READ_TYPE(uint16_t, uint8_t*);

/**
 * @brief Functions of MPU6050_BURST_READ_TYPE should read multiple bytes
 * at the address specified by the first parameter into the memory
 * pointed to by the 2nd parameter.
 * The MPU6050's FIFO R/W register can be read in quick succession
 * to access buffered data.
 * @param 1: Address of the register to read
 * @param 2: Pointer to read the register's data into
 */
typedef int MPU6050_BURST_READ_TYPE(uint16_t, uint8_t*, uint16_t);

/**
 * @brief Functions of MPU6050_REG_WRITE_TYPE should write the data
 * contained in the 2nd parameter to the address specified by
 * the first parameter.
 * at the address specified by the first parameter 
 * @param 1: Address of the register to write
 * @param 2: Data to be written
 */
typedef int MPU6050_REG_WRITE_TYPE(uint16_t, uint8_t);

/**
 * @brief Functions of DELAY_MS_TYPE should delay program
 * execution by the time specified by the first parameter 
 * in milliseconds. Especially when changing config registers 
 * or running self-tests, the MPU6050 can get into a faulty
 * state without waiting for it to catch up.
 * @param 1: Delay time in ms
 */
typedef void DELAY_MS_TYPE(uint32_t);

/**
 * @brief Functions of TIME_MS_TYPE should return the
 * tick time in milliseconds
 */
typedef uint32_t TIME_MS_TYPE(void);

/**
 * @brief Sets configuration registers to default values
 * 1. Reset Power Management Register - 0x6B
 * 2. Reset fifo, i2c master, sensor signal paths, and sensors
 * using User Control register - 0x6A
 * 3. Sets clock source to internal via Power Management Register - 0x6B
 * 4. Awakens all gyro and accel axes via Power Management 2 Register - 0x6C
 *    (turn off unused axes later)
 * @param writeReg function that performs a register write to the MPU6050
 * @param delay function that performs a blocking wait in ms
 */
uint16_t init_mpu6050(MPU6050_REG_WRITE_TYPE writeReg, DELAY_MS_TYPE delay);

/**
 * @brief Accelerometer readings are 2 bytes, stored in two registers on the
 * MPU6050. Reads both, then combines and scales value.
 * @param address most significant accel axis register
 * @param scaler scale the raw measurement to the expected unit
 * @param readReg function that performs register reads on the MPU6050
 * @return The accel measurement in G (9.8m/s^2)
 */
float read_accel_axis(uint8_t address, uint16_t scaler, MPU6050_REG_READ_TYPE readReg);

/**
 * @brief Gyroscope readings are 2 bytes, stored in two registers on the
 * MPU6050. Reads both, then combines and scales value.
 * @param address most significant gyro axis register
 * @param scaler scale the raw measurement to the expected unit
 * @param readReg function that performs register reads on the MPU6050
 * @return The gyro measurement in DPS (degrees per second)
 */
float read_gyro_axis(uint8_t address, uint16_t scaler, MPU6050_REG_READ_TYPE readReg);

/**
 * @brief Get the raw reading from any of the accelerometer or gyro axes
 * @param address: most significant axis register
 * @param readReg function that performs register reads on the MPU6050
 * @return the raw axis measurement 
 */
int16_t read_raw_axis(uint8_t address, MPU6050_REG_READ_TYPE readReg);


/**
 * @brief The factory test functions measure how far off the trim for each
 * axis is from the factory values. If an axis > abs(passPercent), then
 * that axis failed the test. See the self test registers description in the 
 * MPU6050 register map for more information.
 */
typedef struct
{
  float xAxis;
  float yAxis;
  float zAxis;
  float failPercent;  
} FACTORY_TEST_RESULTS;


/**
 * @brief Runs a self test on the gyro. Steps:
 * 1. Set gyro's full scale range to 250dps
 * 2. Save gyro's output with self test disabled (TD)
 * 3. Enable self test register
 * 4. Save gyro's output with self test enabled (TE)
 * 5. SelfTestResponse (STR) = TE - TD
 * 6. Obtain Factory Trim values (FT)
 * 7. Calculate the change from factory trim for each axis
 * 8. Store the change from factory trim in the FACTORY_TEST_RESULTS struct
 * @param readReg function that performs register reads on the MPU6050
 * @param writeReg function that performs register writes on the MPU6050
 * @param delay function that can block program execution
 * @return The change from factory trim expressed as a percentage for each axis
 */
FACTORY_TEST_RESULTS gyro_self_test(MPU6050_REG_READ_TYPE readReg, MPU6050_REG_WRITE_TYPE writeReg, DELAY_MS_TYPE delay);


/**
 * Runs a self test on the accelerometer. Steps:
 * 1. Set accelerometer's full scale range to 8g
 * 2. Save accel output with self test disabled (TD)
 * 3. Enable self test registers
 * 4. Save accel output with self test enabled (TE)
 * 5. SelfTestResponse (STR) = TE - TD
 * 6. Obtain Factory Trim values (FT)
 * 7. Calculate the change from factory trim for each axis
 * 8. Store the change from factory trim in the FACTORY_TEST_RESULTS struct
 * @param readReg function that performs register reads on the MPU6050
 * @param writeReg function that performs register writes on the MPU6050
 * @param delay function that can block program execution
 * @return The change from factory trim expressed as a percentage for each axis
 */
FACTORY_TEST_RESULTS accel_self_test(MPU6050_REG_READ_TYPE readReg, MPU6050_REG_WRITE_TYPE writeReg, DELAY_MS_TYPE delay);


/**
 * @brief Helper function to read the number of bytes currently in the fifo
 * Reads REG_FIFO_COUNT_H first, then REG_FIFO_COUNT_L, and concatenates
 * them
 * @param readReg function that performs register reads on the MPU6050
 * @return The number of bytes currently stored in the MPU6050's fifo buffer
 */
uint16_t read_fifo_count(MPU6050_REG_READ_TYPE readReg);

#ifdef __cplusplus
}
#endif

#endif /* INC_MPU6050_H_ */

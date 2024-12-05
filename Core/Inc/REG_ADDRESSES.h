/*
 * REG_ADDRESSES.h
 *
 *  Created on: Nov 12, 2024
 *      Author: Kade
 */

#ifndef INC_REG_ADDRESSES_H_
#define INC_REG_ADDRESSES_H_

//7 bit address from datasheet. LSB takes value of AD0, which is low
#define MPU_6050_ADDR (0x68)

//HAL docs state datasheet address must be left shifted by 1 before being passed to HAL_I2C functions
#define MPU_6050_HAL_I2C_ADDR (MPU_6050_ADDR << 1)

//Contains the 7 bit address of MPU_6050
#define REG_WHO_AM_I (0x75)

//Power mode and clock source (internal, external, gyro(?))
#define REG_PWR_MGMT_1 (0x6B)
#define PWR_MGMT_DEV_RESET (0x80) //Set all registers to their defaults
#define PWR_MGMT_CLK_SEL_INTERNAL (0x0) //Select the clock source to be the internal 8MHz clock

//Gyro sample rate divider. Sample Rate = Gyro rate / (1 + REG_SMPRT_DIV)
#define REG_SMPRT_DIV (0x19)
#define SAMPLE_RATE_100Hz (0x9) //Gyro rate = 1KHz when DLPF enabled

//Digital low pass and external Frame Synchronization Configuration
#define REG_CONFIG (0x1A)
#define DLPF_OFF (0x0) //makes sample rate 8KHz
#define DLPF_MAX_FILTER (0x6) //makes sample rate 1KHz
#define EXT_SYNC_OFF (0x0) //turn off fsync (not sure what this does)

//Gyroscope config and self test activation register
#define REG_GYRO_CONFIG (0x1B)
#define GYRO_FS_SEL_250_DPS (0x0) //Gyro full scale range +-250 degrees / sec
#define GYRO_FS_SEL_500_DPS (0x1 << 3) //Gyro full scale range +-500 degrees / sec
#define GYRO_FS_SEL_1000_DPS (0x2 << 3) //Gyro full scale range +-1000 degrees / sec
#define GYRO_FS_SEL_2000_DPS (0x3 << 3) //Gyro full scale range +-2000 degrees / sec

//Accelerometer config and self test activation register
#define REG_ACCEL_CONFIG (0x1C)
#define ACCEL_FS_2G (0x0) //Accel full scale range +-2g
#define ACCEL_FS_4G (0x1 << 3) //Accel full scale range +-4g
#define ACCEL_FS_8G (0x2 << 3) //Accel full scale range +-8g
#define ACCEL_FS_16G (0x3 << 3) //Accel full scale range +-16g

#endif /* INC_REG_ADDRESSES_H_ */

/*
 * REG_OPTIONS.h
 *
 *  Created on: Nov 12, 2024
 *      Author: Kade
 * Includes Register addresses and common options used for those registers
 */

#ifndef INC_REG_OPTIONS_H_
#define INC_REG_OPTIONS_H_

//7 bit address from datasheet. LSB takes value of AD0, which is low
#define MPU_6050_ADDR (0x68)

//HAL docs state datasheet address must be left shifted by 1 before being passed to HAL_I2C functions
#define MPU_6050_HAL_I2C_ADDR (MPU_6050_ADDR << 1)

//Contains the 7 bit address of MPU_6050
#define REG_WHO_AM_I (0x75)

#define REG_USER_CTRL (0x6A)
#define FIFO_RESET (0x4)
#define I2C_MST_RESET (0x2)
#define SIG_COND_RESET (0x1)

//Power mode and clock source (internal, external, gyro(?))
#define REG_PWR_MGMT_1 (0x6B)
#define PWR_MGMT_DEV_RESET (0x80) //Set all registers to their defaults
#define PWR_MGMT_CLK_SEL_INTERNAL (0x0) //Select the clock source to be the internal 8MHz clock

//Power management 2
#define REG_PWR_MGMT_2 (0x6C)
// accelerometer low power mode 
#define LP_WAKE_CTRL_0 (0x0) //wake-up 1.25Hz
#define LP_WAKE_CTRL_1 (0x1 << 6) //5 Hz
#define LP_WAKE_CTRL_2 (0x2 << 6) //20Hz
#define LP_WAKE_CTRL_3 (0x3 << 6) //40Hz
//set specific axes of gyro and accel to standby mode
#define STBY_XA (0x1 << 5) //accelerometer x
#define STBY_YA (0x1 << 4)
#define STBY_ZA (0x1 << 3)
#define STBY_XG (0x1 << 2) //gyro x
#define STBY_YG (0x1 << 1)
#define STBY_ZG (0x1 << 0)

/**
 * Gyro sample rate divider: 
 * Sample Rate = Gyro rate / (1 + REG_SMPRT_DIV)
 * Gyro rate is 1KHz, unless the low pass filter (DLPF) is turned
 * off
 */
#define REG_SMPRT_DIV (0x19)
#define SAMPLE_RATE_100Hz (0x9) //Gyro rate = 1KHz when DLPF enabled

/**
 * Digital low pass and external Frame Synchronization Configuration:
 * Setting the low pass filter to DLPF_CFG_0 causes the gyro's
 * sample rate to be 8KHz. Otherwise, the gyro's sample rate is 
 * 1KHz, and the accelerometer's is always 1KHz. See page 13
 * of register map for bandwidth and delay at each DLPF setting.
 */
#define REG_CONFIG (0x1A)
#define DLPF_CFG_0 (0x0) //no filtering
#define DLPF_CFG_1 (0x1) 
#define DLPF_CFG_2 (0x2)
#define DLPF_CFG_3 (0x3)
#define DLPF_CFG_4 (0x4)
#define DLPF_CFG_5 (0x5)
#define DLPF_CFG_6 (0x6) //max filtering
#define EXT_SYNC_OFF (0x0) //turn off fsync

//Gyroscope config and self test activation register
#define REG_GYRO_CONFIG (0x1B)
#define GYRO_FS_SEL_250_DPS (0x0) //Gyro full scale range +-250 degrees / sec
#define GYRO_FS_SEL_500_DPS (0x1 << 3) //Gyro full scale range +-500 degrees / sec
#define GYRO_FS_SEL_1000_DPS (0x2 << 3) //Gyro full scale range +-1000 degrees / sec
#define GYRO_FS_SEL_2000_DPS (0x3 << 3) //Gyro full scale range +-2000 degrees / sec
#define GYRO_FS_250_DIV (131.0f) //gyro scaler for 250 DPS
#define GYRO_FS_500_DIV (65.5f) //gyro scaler for 500 DPS
#define GYRO_FS_1000_DIV (32.8f) //gyro scaler for 1000 DPS
#define GYRO_FS_2000_DIV (16.4f) //gyro scaler for 2000 DPS

//Accelerometer config and self test activation register
#define REG_ACCEL_CONFIG (0x1C)
#define ACCEL_FS_2G (0x0) //Accel full scale range +-2g
#define ACCEL_FS_4G (0x1 << 3) //Accel full scale range +-4g
#define ACCEL_FS_8G (0x2 << 3) //Accel full scale range +-8g
#define ACCEL_FS_16G (0x3 << 3) //Accel full scale range +-16g
#define ACCEL_FS_2_DIV (16384.0f) //accel scaler for 2g
#define ACCEL_FS_4_DIV (8192.0f) //accel scaler for 4g
#define ACCEL_FS_8_DIV (4096.0f) //accel scaler for 8g
#define ACCEL_FS_16_DIV (2048.0f) //accel scaler for 16g

#define REG_ACCEL_X_MEASURE_1 (0x3B) //[15:8]
#define REG_ACCEL_X_MEASURE_2 (0x3C) //[7:0]

#define REG_ACCEL_Y_MEASURE_1 (0x3D) //[15:8]
#define REG_ACCEL_Y_MEASURE_2 (0x3E) //[7:0]

#define REG_ACCEL_Z_MEASURE_1 (0x3F) //[15:8]
#define REG_ACCEL_Z_MEASURE_2 (0x40) //[7:0]

#define REG_GYRO_X_MEASURE_1 (0x43) //[15:8]
#define REG_GYRO_X_MEASURE_2 (0x44) //[7:0]

#define REG_GYRO_Y_MEASURE_1 (0x45) //[15:8]
#define REG_GYRO_Y_MEASURE_2 (0x46) //[7:0]

#define REG_GYRO_Z_MEASURE_1 (0x47) //[15:8]
#define REG_GYRO_Z_MEASURE_2 (0x48) //[7:0]

#endif /* INC_REG_OPTIONS_H_ */
/*
 * MPU6050.c
 *
 *  Created on: Nov 25, 2024
 *      Author: Kade Perrotti
 * Implementations of functions that initialize and read data from
 * the MPU6050
 */
#include <stdlib.h>
#include <math.h>

#include "REG_OPTIONS.h"
#include "MPU6050.h"


uint16_t init_mpu6050(MPU6050_REG_WRITE_TYPE writeReg, DELAY_MS_TYPE delay)
{
    //reset the power managment register
    writeReg(REG_PWR_MGMT_1, PWR_MGMT_DEV_RESET);
    delay(100);

    //Reset fifo, i2c master, sensor signal paths and sensors
    //via user ctrl register
    writeReg(REG_USER_CTRL, I2C_MST_RESET | SIG_COND_RESET);
    delay(100);

    //set clock source
    writeReg(REG_PWR_MGMT_1, PWR_MGMT_CLK_SEL_INTERNAL);

    //setup 2nd power management register
    //MPU6050_REG_WRITE(REG_PWR_MGMT_2, STBY_ZG | STBY_XG | STBY_YG); //gyro sleeps
    writeReg(REG_PWR_MGMT_2, 0x0); //awaken all accel + gyro axes
    
    delay(1000);

    return 0;
}

float read_accel_axis(uint8_t address, uint16_t scaler, MPU6050_REG_READ_TYPE readReg)
{
    int16_t rawReading = read_raw_axis(address, readReg);
    float scaled = (float)rawReading / scaler;
    return scaled;
}

float read_gyro_axis(uint8_t address, uint16_t scaler, MPU6050_REG_READ_TYPE readReg)
{
    int16_t rawReading = read_raw_axis(address, readReg);
    float scaled = (float)rawReading / scaler;
    return scaled;
}

int16_t read_raw_axis(uint8_t address, MPU6050_REG_READ_TYPE readReg)
{
    uint8_t measureUpper = 0;
    uint8_t measureLower = 0;
    
    //upper portion of accel 
    readReg(address, &measureUpper);
    
    //lower portion
    readReg(address + 1, &measureLower);
    
    int16_t combined = (int16_t)(measureUpper << 8) | measureLower;
    return combined;
}

FACTORY_TEST_RESULTS gyro_self_test(MPU6050_REG_READ_TYPE readReg, MPU6050_REG_WRITE_TYPE writeReg, DELAY_MS_TYPE delay)
{
    //save old gyro full scale range
    uint8_t gyroFs = 0;

    readReg(REG_GYRO_CONFIG, &gyroFs);
    
    gyroFs &= GYRO_FS_SEL_MASK; //keep only the FS_SEL setting

    //set gyro to 250 dps for test
    writeReg(REG_GYRO_CONFIG, GYRO_FS_SEL_250_DPS);

    //wait
    delay(250);

    //get gyro's output with self test disabled
    int16_t TD[3]; //3 axis
    TD[0] = read_raw_axis(REG_GYRO_X_MEASURE_1, readReg);
    TD[1] = read_raw_axis(REG_GYRO_Y_MEASURE_1, readReg);
    TD[2] = read_raw_axis(REG_GYRO_Z_MEASURE_1, readReg);

    //enable self test, and datasheet requires gyro set to 250 DPS
    writeReg(
        REG_GYRO_CONFIG, 
        GYRO_FS_SEL_250_DPS | GYRO_XG_ST | GYRO_YG_ST | GYRO_ZG_ST
    );

    //wait
    delay(250);
    
    //get gyro's output with self test enabled
    int16_t TE[3]; //3 axis
    TE[0] = read_raw_axis(REG_GYRO_X_MEASURE_1, readReg);
    TE[1] = read_raw_axis(REG_GYRO_Y_MEASURE_1, readReg);
    TE[2] = read_raw_axis(REG_GYRO_Z_MEASURE_1, readReg);

    //calculate the value of STR from the datasheet. This is
    //different from reading the SELF_TEST (GTest) registers below
    int16_t selfTestResponse[3];
    selfTestResponse[0] = TE[0] - TD[0];
    selfTestResponse[1] = TE[1] - TD[1];
    selfTestResponse[2] = TE[2] - TD[2];
    

    //read self test registers
    uint8_t GTest[3];

    readReg(REG_SELF_TEST_X, &GTest[0]);
    readReg(REG_SELF_TEST_Y, &GTest[1]);
    readReg(REG_SELF_TEST_Z, &GTest[2]);

    GTest[0] &= XG_TEST_MASK;
    GTest[1] &= YG_TEST_MASK;
    GTest[2] &= ZG_TEST_MASK;
    
    //calculate factory trims using self test registers
    float factoryTrim[3];
    factoryTrim[0] = 25.0f * 131.0f * powf(1.046f, (float)GTest[0] - 1.0f);
    factoryTrim[1] = -25.0f * 131.0f * powf(1.046f, (float)GTest[1] - 1.0f); //y axis has -25.0 in datasheet
    factoryTrim[2] = 25.0f * 131.0f * powf(1.046f, (float)GTest[2] - 1.0f);

    //finally, calculate test results
    FACTORY_TEST_RESULTS results;
    results.failPercent = SELF_TEST_FAIL_PERCENT;
    results.xAxis = 100.0f * (((float)selfTestResponse[0] - factoryTrim[0]) / factoryTrim[0]);
    results.yAxis = 100.0f * (((float)selfTestResponse[1] - factoryTrim[1]) / factoryTrim[1]);
    results.zAxis = 100.0f * (((float)selfTestResponse[2] - factoryTrim[2]) / factoryTrim[2]);

    //revert test setup
    writeReg(REG_GYRO_CONFIG, gyroFs);
    
    return results;
}

FACTORY_TEST_RESULTS accel_self_test(MPU6050_REG_READ_TYPE readReg, MPU6050_REG_WRITE_TYPE writeReg, DELAY_MS_TYPE delay)
{
    //save old accel full scale range
    uint8_t accelFs = 0;

    readReg(REG_ACCEL_CONFIG, &accelFs);
    
    accelFs &= ACCEL_FS_SEL_MASK; //keep only the FS_SEL setting

    //set accel to 8g for test
    writeReg(REG_ACCEL_CONFIG, ACCEL_FS_8G);

    //wait
    delay(250);

    //get accels's output with self test disabled
    int16_t TD[3]; //3 axis
    TD[0] = read_raw_axis(REG_ACCEL_X_MEASURE_1, readReg);
    TD[1] = read_raw_axis(REG_ACCEL_Y_MEASURE_1, readReg);
    TD[2] = read_raw_axis(REG_ACCEL_Z_MEASURE_1, readReg);

    //enable self test, and datasheet requires accel set to 8g
    writeReg(
        REG_ACCEL_CONFIG, 
        ACCEL_FS_8G | ACCEL_XA_ST | ACCEL_YA_ST | ACCEL_ZA_ST
    );

    //wait
    delay(250);
    
    //get accels's output with self test enabled
    int16_t TE[3]; //3 axis
    TE[0] = read_raw_axis(REG_ACCEL_X_MEASURE_1, readReg);
    TE[1] = read_raw_axis(REG_ACCEL_Y_MEASURE_1, readReg);
    TE[2] = read_raw_axis(REG_ACCEL_Z_MEASURE_1, readReg);

    //calculate the value of STR from the datasheet. This is
    //different from reading the SELF_TEST (ATest) registers below
    int16_t selfTestResponse[3];
    selfTestResponse[0] = TE[0] - TD[0];
    selfTestResponse[1] = TE[1] - TD[1];
    selfTestResponse[2] = TE[2] - TD[2];
    

    //read self test registers, each axis is 5 bits, split across two registers
    uint8_t ATestUpper[3]; //more significant portion
    uint8_t SELF_TEST_A; //less significant portion

    //upper 3 bits
    readReg(REG_SELF_TEST_X, &ATestUpper[0]);
    readReg(REG_SELF_TEST_Y, &ATestUpper[1]);
    readReg(REG_SELF_TEST_Z, &ATestUpper[2]);
    
    //lower 2 bits
    readReg(REG_SELF_TEST_A, &SELF_TEST_A);

    //final combined accel test values
    uint8_t ATest[3];
    ATest[0] = ((ATestUpper[0] & XA_TEST_UPPER_MASK) >> 3) | ((SELF_TEST_A & XA_TEST_LOWER_MASK) >> 4);
    ATest[1] = ((ATestUpper[1] & YA_TEST_UPPER_MASK) >> 3) | ((SELF_TEST_A & YA_TEST_LOWER_MASK) >> 2);
    ATest[2] = ((ATestUpper[2] & ZA_TEST_UPPER_MASK) >> 3) | (SELF_TEST_A & ZA_TEST_LOWER_MASK);
    
    //calculate factory trims using self test registers
    float factoryTrim[3];
    factoryTrim[0] = 4096.0f * 0.34f * powf((0.92f/0.34f), ((float)ATest[0] - 1) / 30.0f);
    factoryTrim[1] = 4096.0f * 0.34f * powf((0.92f/0.34f), ((float)ATest[1] - 1) / 30.0f);
    factoryTrim[2] = 4096.0f * 0.34f * powf((0.92f/0.34f), ((float)ATest[1] - 1) / 30.0f);

    //finally, calculate test results
    FACTORY_TEST_RESULTS results;
    results.failPercent = SELF_TEST_FAIL_PERCENT;
    results.xAxis = 100.0f * (((float)selfTestResponse[0] - factoryTrim[0]) / factoryTrim[0]);
    results.yAxis = 100.0f * (((float)selfTestResponse[1] - factoryTrim[1]) / factoryTrim[1]);
    results.zAxis = 100.0f * (((float)selfTestResponse[2] - factoryTrim[2]) / factoryTrim[2]);

    //revert test setup
    writeReg(REG_ACCEL_CONFIG, accelFs);
    
    return results;
}

uint16_t read_fifo_count(MPU6050_REG_READ_TYPE readReg)
{
    uint8_t upper = 0;
    uint8_t lower = 0;
    readReg(REG_FIFO_COUNT_H, &upper);
    readReg(REG_FIFO_COUNT_L, &lower);
    uint16_t combined = (((uint16_t)upper) << 8) | lower;
    return combined;
}

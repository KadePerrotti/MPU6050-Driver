/*
 * TEST_FUNCTIONS.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Kade Perrotti
 * Tests functions to ensure normal MPU6050 operation
 */

#include "TEST_FUNCTIONS.h"
#include "REG_OPTIONS.h"


SETUP_REGISTERS read_setup_registers(MPU6050_REG_READ_TYPE readReg)
{
    SETUP_REGISTERS vals; //
    uint8_t readBuff; //actual register value goes here

    //config register
    readReg(REG_CONFIG, &readBuff);
    vals.fsync = readBuff & FSYNC_MASK;
    vals.dlpf = readBuff & DLPF_MASK;
    readBuff = 0;

    //gyro config
    readReg(REG_GYRO_CONFIG, &readBuff);
    vals.gyro_sel = readBuff & GYRO_FS_SEL_MASK;
    readBuff = 0;

    //accelerometer config register
    readReg(REG_ACCEL_CONFIG, &readBuff);
    vals.fs = readBuff & ACCEL_FS_SEL_MASK;
    readBuff = 0;

    //sample rate divider
    readReg(REG_SMPRT_DIV, &readBuff);
    vals.rate_div = readBuff & RATE_DIV_MASK;
    readBuff = 0;

    return vals;
}

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
)
{
    const int numAxes = 6;
    float samplePeriodMs = S_TO_MS((1.0f / sampleRate));
    int numSamples = numAxes * MS_TO_S((float)sampleTime) * sampleRate;
    int i = 0;
    while (i < numSamples)
    {
        int startRead = getTime();
        float accelX = read_accel_axis(REG_ACCEL_X_MEASURE_1, accelScaler, readReg);
        data[i++] = accelX;
        
        float accelY = read_accel_axis(REG_ACCEL_Y_MEASURE_1, accelScaler, readReg);
        data[i++] = accelY;
        
        float accelZ = read_accel_axis(REG_ACCEL_Z_MEASURE_1, accelScaler, readReg);
        data[i++] = accelZ;

        float gyroX = read_gyro_axis(REG_GYRO_X_MEASURE_1, gyroScaler, readReg);
        data[i++] = gyroX;

        float gyroY = read_gyro_axis(REG_GYRO_Y_MEASURE_1, gyroScaler, readReg);
        data[i++] = gyroY;

        float gyroZ = read_gyro_axis(REG_GYRO_Z_MEASURE_1, gyroScaler, readReg);
        data[i++] = gyroZ;


        int endRead = getTime();
        int totalTime = endRead - startRead; //ms
        delay(samplePeriodMs - totalTime);
    }
}
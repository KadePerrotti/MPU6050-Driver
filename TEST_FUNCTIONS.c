/*
 * TEST_FUNCTIONS.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Kade Perrotti
 * Tests functions to ensure normal MPU6050 operation, and string
 * builder functions for printing test results
 */
#include <stdio.h>
#include "TEST_FUNCTIONS.h"
#include "REG_OPTIONS.h"


SETUP_REGISTERS read_setup_registers(MPU6050_REG_READ_TYPE readReg)
{
    SETUP_REGISTERS vals; //configuration values to return
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
    vals.accel_sel = readBuff & ACCEL_FS_SEL_MASK;
    readBuff = 0;

    //sample rate divider
    readReg(REG_SMPRT_DIV, &readBuff);
    vals.rate_div = readBuff & RATE_DIV_MASK;
    readBuff = 0;

    return vals;
}

uint16_t print_setup_registers_results(SETUP_REGISTERS expected, SETUP_REGISTERS actual, char* buff)
{
  uint16_t size = 0; //size of the string being built
  size += sprintf(
        buff + size, 
        "\r\n\n Config Reg: \r\n  FSYNC: %d, %c\r\n  DLPF: %d, %c\r\n", 
        actual.fsync, 
        actual.fsync == expected.fsync ? 't' : 'f',
        actual.dlpf,
        actual.dlpf == expected.dlpf ? 't' : 'f');

  size += sprintf(
        buff + size, 
        "\r\n\n Gyro Config Reg: \r\n  Full Scale: %d, %c\r\n", 
        actual.gyro_sel, 
        actual.gyro_sel == expected.gyro_sel ? 't' : 'f'
        );
  
  size += sprintf(
        buff + size, 
        "\r\n\n Accel Config Reg: \r\n  Full Scale: %d, %c\r\n", 
        actual.accel_sel, 
        actual.accel_sel == expected.accel_sel ? 't' : 'f'
    );

  size += sprintf(
        buff + size, 
        "\r\n\n Sample Rate Div: \r\n  Divider: %d, %c\r\n", 
        actual.rate_div, 
        actual.rate_div == expected.rate_div ? 't' : 'f'
    );
    buff[size] = '\0';
    return size;
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

uint16_t print_self_test_results(FACTORY_TEST_RESULTS gyroResults, FACTORY_TEST_RESULTS accelResults, char* buff)
{
    uint16_t size = 0; //size of the string being built

    // Gyro
    size += sprintf(
        buff + size,
        "\r\nGyro X self test: %c, change from factory trim: %f%%", 
        ( gyroResults.failPercent > gyroResults.xAxis && gyroResults.xAxis > -gyroResults.failPercent ) ? 'P' : 'F', 
        gyroResults.xAxis
    );
    

    size += sprintf(
        buff + size,
        "\r\nGyro Y self test: %c, change from factory trim: %f%%", 
        ( gyroResults.failPercent > gyroResults.yAxis && gyroResults.yAxis > -gyroResults.failPercent ) ? 'P' : 'F', 
        gyroResults.yAxis
    );

    size += sprintf(
        buff + size,
        "\r\nGyro Z self test: %c, change from factory trim: %f%%", 
        ( gyroResults.failPercent > gyroResults.zAxis && gyroResults.zAxis > -gyroResults.failPercent ) ? 'P' : 'F', 
        gyroResults.zAxis
    );

    // Accel
    size += sprintf(
        buff + size,
        "\r\nAccel X self test: %c, change from factory trim: %f%%", 
        ( accelResults.failPercent > accelResults.xAxis && accelResults.xAxis > -accelResults.failPercent ) ? 'P' : 'F', 
        accelResults.xAxis
    );

    size += sprintf(
        buff + size,
        "\r\nAccel Y self test: %c, change from factory trim: %f%%", 
        ( accelResults.failPercent > accelResults.yAxis && accelResults.yAxis > -accelResults.failPercent ) ? 'P' : 'F', 
        accelResults.yAxis
    );

    size += sprintf(
        buff + size,
        "\r\nAccel Z self test: %c, change from factory trim: %f%%", 
        ( accelResults.failPercent > accelResults.zAxis && accelResults.zAxis > -accelResults.failPercent ) ? 'P' : 'F', 
        accelResults.zAxis
    );
    return size;
}

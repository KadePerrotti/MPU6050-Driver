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

uint16_t build_setup_registers_string(SETUP_REGISTERS expected, SETUP_REGISTERS actual, char* buff)
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

uint16_t build_poll_axes_string(float *data, uint16_t dataSize, char *buff)
{
    uint16_t totalSize = 0;
    const int numAxis = 6;
    
    // build table header
    totalSize += sprintf(buff + totalSize, "\r\n\naccelX, accelY, accelZ, gyroX, gyroY, gyroZ");

    // build data rows
    for (uint16_t i = 0; i < dataSize; i += numAxis)
    {
        totalSize += sprintf(buff + totalSize, "\r\n%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", 
            data[i], data[i + 1], data[i + 2], // accel xyz
            data[i + 3], data[i + 4], data[i + 5]  // gyro xyz
        );
    }

    return totalSize;
}


uint16_t build_self_tests_string(FACTORY_TEST_RESULTS gyroResults, FACTORY_TEST_RESULTS accelResults, char* buff)
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
)
{
    const uint8_t numTests = NUM_FIFO_COUNT_TESTS;

    // Calculate expected bytes per read period
    *bytesPerRead = (BYTES_PER_MEASURE * sampleRate * numAxes * readPeriodMs / 1000.0f);
    
    // Ensure expected bytes do not exceed FIFO size
    if (*bytesPerRead > FIFO_SIZE)
    {
        return; // Caller should handle error reporting
    }

    // Read data into this buffer 
    uint8_t throwAway[1024]; 
    
    // Clear the buffer and count by reading the FIFO
    uint16_t clearCount = read_fifo_count(readReg); // Read the current FIFO count
    burstRead(REG_FIFO_R_W, throwAway, clearCount); // Discard the read values
    
    delay(readPeriodMs); // Wait for the required time
    
    // Read FIFO count numTests times
    for (int i = 0; i < numTests; i++)
    {
        uint32_t startTime = getTime();
        
        fifoCountResults[i] = read_fifo_count(readReg); // Store FIFO count
        
        // Clear FIFO by reading it
        burstRead(REG_FIFO_R_W, throwAway, fifoCountResults[i]);
        
        uint32_t endTime = getTime();
        timePerIter[i] = endTime - startTime;

        // Delay to allow FIFO to refill
        delay(readPeriodMs - timePerIter[i]); 
    }
}

uint16_t fifo_count_build_string(
    uint16_t *fifoCountResults, 
    uint32_t *timePerIter, 
    float bytesPerRead, 
    char *buff
)
{
    uint16_t size = 0;
    const uint8_t numTests = NUM_FIFO_COUNT_TESTS;

    //first, ensure the expected bytes per read did not overflow fifo
    if (bytesPerRead > FIFO_SIZE)
    {
        size += sprintf(buff + size, "\r\n\nError: Expected bytesPerRead (%.2f) is larger than fifo", bytesPerRead);
        return size;
    }

    // Header message
    size += sprintf(buff + size, "\r\n\nExpecting %.2f bytes in fifo each test", bytesPerRead);

    // Append test results
    for (int i = 0; i < numTests; i++)
    {
        size += sprintf(buff + size, 
            "\r\ntest %d, counted %d bytes in fifo, took %lu ms to read fifo",
            i + 1,
            fifoCountResults[i],
            timePerIter[i] 
        );
    }

    return size;
}

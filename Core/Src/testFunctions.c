/*
 * testFunctions.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Kade
 */
#include <stdio.h>

#include "usart.h"

#include "testFunctions.h"
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

void poll_axes_individually(MPU6050_REG_READ_TYPE readReg)
{
    char txBuff[100];
    int numSamples = 6 * 5 * 100; //6 axis * 5 seconds * 100 samples/sec
    float samples[numSamples];
    uint8_t buffLen = 0;
    const float samplingFreq = 100; //100Hz freq
    const float samplingPeriod = 1.0f / samplingFreq; //0.01s
    const int samplingPeriodMs = (int)S_TO_MS(samplingPeriod);
    buffLen = sprintf(txBuff, "\r\naccelX, accelY, accelZ, gyroX, gyroY, gyroZ");
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, buffLen, 100);
    txBuff[0] = '\0';
    int i = 0;
    while (i < numSamples)
    {
        /* USER CODE END WHILE */
        int startRead = HAL_GetTick();
        float accelX = read_accel_axis(REG_ACCEL_X_MEASURE_1, ACCEL_FS_2_DIV, readReg);
        samples[i++] = accelX;
        
        float accelY = read_accel_axis(REG_ACCEL_Y_MEASURE_1, ACCEL_FS_2_DIV, readReg);
        samples[i++] = accelY;
        
        float accelZ = read_accel_axis(REG_ACCEL_Z_MEASURE_1, ACCEL_FS_2_DIV, readReg);
        samples[i++] = accelZ;

        float gyroX = read_gyro_axis(REG_GYRO_X_MEASURE_1, GYRO_FS_250_DIV, readReg);
        samples[i++] = gyroX;

        float gyroY = read_gyro_axis(REG_GYRO_Y_MEASURE_1, GYRO_FS_250_DIV, readReg);
        samples[i++] = gyroY;

        float gyroZ = read_gyro_axis(REG_GYRO_Z_MEASURE_1, GYRO_FS_250_DIV, readReg);
        samples[i++] = gyroZ;


        int endRead = HAL_GetTick();
        int totalTime = endRead - startRead; //ms
        HAL_Delay(samplingPeriodMs - totalTime);
    }
    //write data out to uart
    i = 0;
    while(i < numSamples)
    {
        buffLen = sprintf(txBuff, "\r\n%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", 
            samples[i], samples[i + 1], samples[i + 2], //accel xyz
            samples[i + 3], samples[i + 4], samples[i + 5] //gyro xyz
        );
        HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, buffLen, 100);
        txBuff[0] = '\0';
        i += 6;
    }
}

void fifo_count_test(uint16_t readPeriodMs, uint16_t sampleRate, uint8_t numAxes, MPU6050_BURST_READ_TYPE burstRead, MPU6050_REG_READ_TYPE readReg)
{
    int buffLen = 0; //uart buff len
    char txBuff[100]; //write characters here
    const uint8_t numTests = 10;    
    uint16_t fifo_count_results[numTests];
    uint32_t timePerIter[numTests];

    //number of bytes expected to accumulate in the fifo each read period
    const float numBytesExpected = (BYTES_PER_MEASURE * sampleRate * numAxes * readPeriodMs / 1000.0f);
    
    //ensure the number of expected bytes per read does not exceed fifo size
    if(numBytesExpected > FIFO_SIZE)
    {
        //write results out to uart
        buffLen = sprintf(txBuff, "\r\n\nError: Expected bytes (%.2f) exceeds fifo size (1024)", numBytesExpected);
        HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, buffLen, 100);
        txBuff[0] = '\0';
        return;
    }

    //read data into this buffer 
    uint8_t throwAway[1024]; 
    
    //clear the buffer and count by reading the fifo
    uint16_t clearCount = read_fifo_count(readReg); //only read the buffer for as much data it currently has
    burstRead(REG_FIFO_R_W, throwAway, clearCount); //don't care about values we read
    
    HAL_Delay(readPeriodMs); //delay for the required time
    
    //read count 10 times
    for(int i = 0; i < numTests; i++)
    {
        uint32_t startTime = HAL_GetTick();
        
        fifo_count_results[i] = read_fifo_count(readReg); //save the count
        
        //clear the buffer and count by reading the fifo
        burstRead(REG_FIFO_R_W, throwAway, fifo_count_results[i]);
        
        uint32_t endTime = HAL_GetTick();
        uint32_t total = endTime - startTime;
        timePerIter[i] = total;
        HAL_Delay(readPeriodMs - total); //delay to re-fill the fifo before next read
    }

    //write results out to uart
    buffLen = sprintf(txBuff, "\r\n\nExpecting %.2f bytes in fifo each test", numBytesExpected);
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, buffLen, 100);
    txBuff[0] = '\0';
    for(int i = 0; i < numTests; i++)
    {
        buffLen = sprintf(txBuff, 
            "\r\ntest %d, counted %d bytes in fifo, took %lu ms to read fifo",
            i + 1,
            fifo_count_results[i],
            timePerIter[i] 
        );
        HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, buffLen, 100);
        txBuff[0] = '\0';
    }
}

void read_fifo_test(uint16_t readPeriodMs, MPU6050_BURST_READ_TYPE burstRead, MPU6050_REG_READ_TYPE readReg)
{
    int buffLen = 0; //uart buff len
    char txBuff[100]; //write characters here
    const uint8_t numTests = 5;
    uint8_t data[numTests][FIFO_SIZE]; // first index is test number, 2nd is actual data
    uint16_t fifo_count_results[numTests]; //save for printing
    const uint8_t numAxes = 6;

    //get data
    for(int i = 0; i < numTests; i++)
    {
        uint32_t startTime = HAL_GetTick();
        
        fifo_count_results[i] = read_fifo_count(readReg); //save the count
        
        //clear the buffer and count by reading the fifo
        burstRead(REG_FIFO_R_W, data[i], fifo_count_results[i]);
        
        uint32_t endTime = HAL_GetTick();
        uint32_t total = endTime - startTime;
        HAL_Delay(readPeriodMs - total); //delay to re-fill the fifo before next read
    }

    //print data
    buffLen = sprintf(txBuff, "\r\n\naccelX, accelY, accelZ, gyroX, gyroY, gyroZ");
            HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, buffLen, 100);
            txBuff[0] = '\0';
    for(int testNum = 0; testNum < numTests; testNum++)
    {
        for(int dataPtr = 0; dataPtr < fifo_count_results[testNum]; dataPtr += (numAxes * 2))
        {
            float accelX = TRANSFORM(data[testNum][dataPtr], data[testNum][dataPtr + 1], ACCEL_FS_2_DIV);
            float accelY = TRANSFORM(data[testNum][dataPtr + 2], data[testNum][dataPtr + 3], ACCEL_FS_2_DIV);
            float accelZ = TRANSFORM(data[testNum][dataPtr + 4], data[testNum][dataPtr + 5], ACCEL_FS_2_DIV);

            float gyroX = TRANSFORM(data[testNum][dataPtr + 6], data[testNum][dataPtr + 7], GYRO_FS_250_DIV);
            float gyroY = TRANSFORM(data[testNum][dataPtr + 8], data[testNum][dataPtr + 9], GYRO_FS_250_DIV);
            float gyroZ = TRANSFORM(data[testNum][dataPtr + 10], data[testNum][dataPtr + 11], GYRO_FS_250_DIV);
            
            buffLen = sprintf(
                txBuff, 
                "\r\n%.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
                accelX, accelY, accelZ, gyroX, gyroY, gyroZ
            );
            HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, buffLen, 100);
            txBuff[0] = '\0';
        }
    }
}
/*
 * utils.c
 *
 *  Created on: Nov 25, 2024
 *      Author: Kade
 */
#include <MPU6050.h>

void MPU6050_REG_WRITE(uint16_t regAddr, uint8_t regValue)
{
    //todo: return the hal status
    HAL_I2C_Mem_Write(
        &hi2c1,
        MPU_6050_HAL_I2C_ADDR,
        regAddr,
        SIZE_1_BYTE,
        &regValue,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
}

void MPU6050_REG_READ(uint16_t regAddr, uint8_t* valAddr)
{
    //todo: return the hal status
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        regAddr,
        SIZE_1_BYTE,
        valAddr,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
}

void MPU6050_BURST_READ(uint16_t regAddr, uint8_t* data, uint16_t bytes)
{
     //todo: return the hal status
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        regAddr,
        SIZE_1_BYTE,
        data,
        bytes,
        HAL_I2C_TIMEOUT
    );
}

uint16_t init_mpu6050(void)
{
    //reset the power managment register
    MPU6050_REG_WRITE(REG_PWR_MGMT_1, PWR_MGMT_DEV_RESET);
    HAL_Delay(100);

    //Reset fifo, i2c master, sensor signal paths and sensors
    //via user ctrl register
    MPU6050_REG_WRITE(REG_USER_CTRL, I2C_MST_RESET | SIG_COND_RESET);
    HAL_Delay(100);

    //set clock source
    MPU6050_REG_WRITE(REG_PWR_MGMT_1, PWR_MGMT_CLK_SEL_INTERNAL);

    //setup 2nd power management register
    //MPU6050_REG_WRITE(REG_PWR_MGMT_2, STBY_ZG | STBY_XG | STBY_YG); //gyro sleeps
    MPU6050_REG_WRITE(REG_PWR_MGMT_2, 0x0); //awaken all accel + gyro axes
    
    HAL_Delay(1000);

    return 0;
}

void read_setup_registers(void)
{
    uint8_t readBuff = 0;
    char txBuff[100];
    int uart_buf_len = 0;

    //config register
    MPU6050_REG_READ(REG_CONFIG, &readBuff);
    uint8_t fsync_val = readBuff & 0x38;
    uint8_t dlpf_val = readBuff & 0x7;
    uart_buf_len = sprintf(
        txBuff, 
        "\r\n\n Config Reg: \r\n  FSYNC: %d, %c\r\n  DLPF: %d, %c\r\n", 
        fsync_val, 
        fsync_val == EXT_SYNC_OFF ? 't' : 'f',
        dlpf_val,
        dlpf_val == DLPF_CFG_6 ? 't' : 'f'
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, uart_buf_len, 100);
    txBuff[0] = '\0';
    readBuff = 0;

    //gyro config
    MPU6050_REG_READ(REG_GYRO_CONFIG, &readBuff);
    uint8_t gyro_sel = readBuff & 0x18;
    uart_buf_len = sprintf(
        txBuff, 
        "\r\n\n Gyro Full Scale: \r\n  FS: %d, %c\r\n", 
        gyro_sel, 
        gyro_sel == GYRO_FS_SEL_250_DPS ? 't' : 'f'
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, uart_buf_len, 100);
    txBuff[0] = '\0';
    readBuff = 0;

    //accelerometer config register
    MPU6050_REG_READ(REG_ACCEL_CONFIG, &readBuff);
    uint8_t fs_val = readBuff & 0x18;
    uart_buf_len = sprintf(
        txBuff, 
        "\r\n\n Accel Config Reg: \r\n  Full Scale: %d, %c\r\n", 
        fs_val, 
        fs_val == ACCEL_FS_2G ? 't' : 'f'
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, uart_buf_len, 100);
    txBuff[0] = '\0';
    readBuff = 0;

    //sample rate divider
    MPU6050_REG_READ(REG_SMPRT_DIV, &readBuff);
    uint8_t rate_div = readBuff & 0xFF;
    uart_buf_len = sprintf(
        txBuff, 
        "\r\n\n Gyro SR Div: \r\n  Divider: %d, %c\r\n", 
        rate_div, 
        rate_div == SAMPLE_RATE_100Hz ? 't' : 'f'
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, uart_buf_len, 100);
    txBuff[0] = '\0';
    readBuff = 0;
}


float read_accel_axis(uint8_t address, uint16_t scaler)
{
    int16_t rawReading = read_raw_axis(address);
    float scaled = (float)rawReading / scaler;
    return scaled;
}

float read_gyro_axis(uint8_t address, uint16_t scaler)
{
    int16_t rawReading = read_raw_axis(address);
    float scaled = (float)rawReading / scaler;
    return scaled;
}

int16_t read_raw_axis(uint8_t address)
{
    uint8_t measureUpper = 0;
    uint8_t measureLower = 0;
    
    //upper portion of accel 
    MPU6050_REG_READ(address, &measureUpper);
    
    //lower portion
    MPU6050_REG_READ(address + 1, &measureLower);
    
    int16_t combined = (int16_t)(measureUpper << 8) | measureLower;
    return combined;
}


/**
 * Runs a self test on the gyro. Steps:
 * 1. Set gyro's full scale range to 250dps
 * 1. Save gyro's output with self test disabled (TD)
 * 2. Enable self test register
 * 3. Save gyro's output with self test enabled (TE)
 * 4. SelfTestResponse (STR) = TE - TD
 * 5. Get Factory Trim from G_Test register
 * 6. Check if gyro passes self test
 * 7. Revert gyroFS setting and turn off self tests
 */
FACTORY_TEST_RESULT gyro_self_test(void)
{
    //save old gyro full scale range
    uint8_t gyroFs = 0;

    MPU6050_REG_READ(REG_GYRO_CONFIG, &gyroFs);
    
    gyroFs &= GYRO_FS_SEL_MASK; //keep only the FS_SEL setting

    //set gyro to 250 dps for test
    MPU6050_REG_WRITE(REG_GYRO_CONFIG, GYRO_FS_SEL_250_DPS);

    //wait
    HAL_Delay(250);

    //get gyro's output with self test disabled
    int16_t TD[3]; //3 axis
    TD[0] = read_raw_axis(REG_GYRO_X_MEASURE_1);
    TD[1] = read_raw_axis(REG_GYRO_Y_MEASURE_1);
    TD[2] = read_raw_axis(REG_GYRO_Z_MEASURE_1);

    //enable self test, and datasheet requires gyro set to 250 DPS
    MPU6050_REG_WRITE(
        REG_GYRO_CONFIG, 
        GYRO_FS_SEL_250_DPS | GYRO_XG_ST | GYRO_YG_ST | GYRO_ZG_ST
    );

    //wait
    HAL_Delay(250);
    
    //get gyro's output with self test enabled
    int16_t TE[3]; //3 axis
    TE[0] = read_raw_axis(REG_GYRO_X_MEASURE_1);
    TE[1] = read_raw_axis(REG_GYRO_Y_MEASURE_1);
    TE[2] = read_raw_axis(REG_GYRO_Z_MEASURE_1);

    //calculate the value of STR from the datasheet. This is
    //different from reading the SELF_TEST (GTest) registers below
    int16_t selfTestResponse[3];
    selfTestResponse[0] = TE[0] - TD[0];
    selfTestResponse[1] = TE[1] - TD[1];
    selfTestResponse[2] = TE[2] - TD[2];
    

    //read self test registers
    uint8_t GTest[3];

    MPU6050_REG_READ(REG_SELF_TEST_X, &GTest[0]);
    MPU6050_REG_READ(REG_SELF_TEST_Y, &GTest[1]);
    MPU6050_REG_READ(REG_SELF_TEST_Z, &GTest[2]);

    GTest[0] &= XG_TEST_MASK;
    GTest[1] &= YG_TEST_MASK;
    GTest[2] &= ZG_TEST_MASK;
    
    //calculate factory trims using self test registers
    float factoryTrim[3];
    factoryTrim[0] = 25.0f * 131.0f * powf(1.046f, (float)GTest[0] - 1.0f);
    factoryTrim[1] = -25.0f * 131.0f * powf(1.046f, (float)GTest[1] - 1.0f); //y axis has -25.0 in datasheet
    factoryTrim[2] = 25.0f * 131.0f * powf(1.046f, (float)GTest[2] - 1.0f);

    //finally, calculate test results
    float testResults[3];
    testResults[0] = 100.0f * (((float)selfTestResponse[0] - factoryTrim[0]) / factoryTrim[0]);
    testResults[1] = 100.0f * (((float)selfTestResponse[1] - factoryTrim[1]) / factoryTrim[1]);
    testResults[2] = 100.0f * (((float)selfTestResponse[2] - factoryTrim[2]) / factoryTrim[2]);
    
    //report test results
    char buff[100];
    uint8_t buffSize = 0;

    buffSize = sprintf(
        buff,
        "\r\nGyro X self test: %c, change from factory trim: %f%%", 
        14.0f > testResults[0] && testResults[0] > -14.0f ? 'P' : 'F' , 
        testResults[0]
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)buff, buffSize, 100);
    buff[0] = '\0';

    buffSize = sprintf(
        buff,
        "\r\nGyro Y self test: %c, change from factory trim: %f%%", 
        14.0f > testResults[1] && testResults[1] > -14.0f ? 'P' : 'F' , 
        testResults[1]
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)buff, buffSize, 100);
    buff[0] = '\0';

    buffSize = sprintf(
        buff,
        "\r\nGyro Z self test: %c, change from factory trim: %f%%", 
        14.0f > testResults[2] && testResults[2] > -14.0f ? 'P' : 'F' , 
        testResults[2]
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)buff, buffSize, 100);
    buff[0] = '\0';


    //revert test setup
    MPU6050_REG_WRITE(REG_GYRO_CONFIG, gyroFs);

    
    return FACTORY_TEST_PASS;
}

FACTORY_TEST_RESULT accel_self_test(void)
{
    //save old accel full scale range
    uint8_t accelFs = 0;

    MPU6050_REG_READ(REG_ACCEL_CONFIG, &accelFs);
    
    accelFs &= ACCEL_FS_SEL_MASK; //keep only the FS_SEL setting

    //set accel to 8g for test
    MPU6050_REG_WRITE(REG_ACCEL_CONFIG, ACCEL_FS_8G);

    //wait
    HAL_Delay(250);

    //get accels's output with self test disabled
    int16_t TD[3]; //3 axis
    TD[0] = read_raw_axis(REG_ACCEL_X_MEASURE_1);
    TD[1] = read_raw_axis(REG_ACCEL_Y_MEASURE_1);
    TD[2] = read_raw_axis(REG_ACCEL_Z_MEASURE_1);

    //enable self test, and datasheet requires accel set to 8g
    MPU6050_REG_WRITE(
        REG_ACCEL_CONFIG, 
        ACCEL_FS_8G | ACCEL_XA_ST | ACCEL_YA_ST | ACCEL_ZA_ST
    );

    //wait
    HAL_Delay(250);
    
    //get accels's output with self test enabled
    int16_t TE[3]; //3 axis
    TE[0] = read_raw_axis(REG_ACCEL_X_MEASURE_1);
    TE[1] = read_raw_axis(REG_ACCEL_Y_MEASURE_1);
    TE[2] = read_raw_axis(REG_ACCEL_Z_MEASURE_1);

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
    MPU6050_REG_READ(REG_SELF_TEST_X, &ATestUpper[0]);
    MPU6050_REG_READ(REG_SELF_TEST_Y, &ATestUpper[1]);
    MPU6050_REG_READ(REG_SELF_TEST_Z, &ATestUpper[2]);
    
    //lower 2 bits
    MPU6050_REG_READ(REG_SELF_TEST_A, &SELF_TEST_A);

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
    float testResults[3];
    testResults[0] = 100.0f * (((float)selfTestResponse[0] - factoryTrim[0]) / factoryTrim[0]);
    testResults[1] = 100.0f * (((float)selfTestResponse[1] - factoryTrim[1]) / factoryTrim[1]);
    testResults[2] = 100.0f * (((float)selfTestResponse[2] - factoryTrim[2]) / factoryTrim[2]);
    
    //report test results
    char buff[100];
    uint8_t buffSize = 0;

    buffSize = sprintf(
        buff,
        "\r\nAccel X self test: %c, change from factory trim: %f%%", 
        14.0f > testResults[0] && testResults[0] > -14.0f ? 'P' : 'F' , 
        testResults[0]
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)buff, buffSize, 100);
    buff[0] = '\0';

    buffSize = sprintf(
        buff,
        "\r\nAccel Y self test: %c, change from factory trim: %f%%", 
        14.0f > testResults[1] && testResults[1] > -14.0f ? 'P' : 'F' , 
        testResults[1]
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)buff, buffSize, 100);
    buff[0] = '\0';

    buffSize = sprintf(
        buff,
        "\r\nAccel Z self test: %c, change from factory trim: %f%%", 
        14.0f > testResults[2] && testResults[2] > -14.0f ? 'P' : 'F' , 
        testResults[2]
    );
    HAL_UART_Transmit(&huart2, (uint8_t*)buff, buffSize, 100);
    buff[0] = '\0';


    //revert test setup
    MPU6050_REG_WRITE(REG_ACCEL_CONFIG, accelFs);

    
    return FACTORY_TEST_PASS;
}


void poll_axes_individually(void)
{
    char txBuff[100];
    int numSamples = 6 * 5 * 100; //6 axis * 5 seconds * 100 samples/sec
    float samples[numSamples];
    uint8_t buffLen = 0;
    const float samplingFreq = 100; //100Hz freq
    const float samplingPeriod = 1.0f / samplingFreq; //0.01s
    const int samplingPeriodMs = (int)S_TO_MS(samplingPeriod);
    buffLen = sprintf(txBuff, "\r\naccelX,accelY,accelZ,gyroX,gyroY,gyroZ");
    HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, buffLen, 100);
    txBuff[0] = '\0';
    int i = 0;
    while (i < numSamples)
    {
        /* USER CODE END WHILE */
        int startRead = HAL_GetTick();
        float accelX = read_accel_axis(REG_ACCEL_X_MEASURE_1, ACCEL_FS_2_DIV);
        samples[i++] = accelX;
        
        float accelY = read_accel_axis(REG_ACCEL_Y_MEASURE_1, ACCEL_FS_2_DIV);
        samples[i++] = accelY;
        
        float accelZ = read_accel_axis(REG_ACCEL_Z_MEASURE_1, ACCEL_FS_2_DIV);
        samples[i++] = accelZ;

        float gyroX = read_gyro_axis(REG_GYRO_X_MEASURE_1, GYRO_FS_250_DIV);
        samples[i++] = gyroX;

        float gyroY = read_gyro_axis(REG_GYRO_Y_MEASURE_1, GYRO_FS_250_DIV);
        samples[i++] = gyroY;

        float gyroZ = read_gyro_axis(REG_GYRO_Z_MEASURE_1, GYRO_FS_250_DIV);
        samples[i++] = gyroZ;


        int endRead = HAL_GetTick();
        int totalTime = endRead - startRead; //ms
        HAL_Delay(samplingPeriodMs - totalTime);
    }
    //write data out to uart
    i = 0;
    while(i < numSamples)
    {
        buffLen = sprintf(txBuff, "\r\n%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", 
            samples[i], samples[i + 1], samples[i + 2], //accel xyz
            samples[i + 3], samples[i + 4], samples[i + 5] //gyro xyz
        );
        HAL_UART_Transmit(&huart2, (uint8_t*)txBuff, buffLen, 100);
        txBuff[0] = '\0';
        i += 6;
    }
}

uint16_t read_fifo_count()
{
    uint8_t upper = 0;
    uint8_t lower = 0;
    MPU6050_REG_READ(REG_FIFO_COUNT_H, &upper);
    MPU6050_REG_READ(REG_FIFO_COUNT_L, &lower);
    uint16_t combined = (((uint16_t)upper) << 8) | lower;
    return combined;
}

void fifo_count_test(uint16_t readPeriodMs, uint16_t sampleRate, uint8_t numAxes)
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
    uint16_t clearCount = read_fifo_count(); //only read the buffer for as much data it currently has
    MPU6050_BURST_READ(REG_FIFO_R_W, throwAway, clearCount); //don't care about values we read
    
    HAL_Delay(readPeriodMs); //delay for the required time
    
    //read count 10 times
    for(int i = 0; i < numTests; i++)
    {
        uint32_t startTime = HAL_GetTick();
        
        fifo_count_results[i] = read_fifo_count(); //save the count
        
        //clear the buffer and count by reading the fifo
        MPU6050_BURST_READ(REG_FIFO_R_W, throwAway, fifo_count_results[i]);
        
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


void read_fifo_test(uint16_t readPeriodMs)
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
        
        fifo_count_results[i] = read_fifo_count(); //save the count
        
        //clear the buffer and count by reading the fifo
        MPU6050_BURST_READ(REG_FIFO_R_W, data[i], fifo_count_results[i]);
        
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

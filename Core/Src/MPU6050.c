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

uint16_t init_mpu6050(void)
{
    //reset the power managment register
    MPU6050_REG_WRITE(REG_PWR_MGMT_1, PWR_MGMT_DEV_RESET);
    HAL_Delay(100);

    //Reset fifo, i2c master, sensor signal paths and sensors
    //via user ctrl register
    MPU6050_REG_WRITE(REG_USER_CTRL, FIFO_RESET | I2C_MST_RESET | SIG_COND_RESET);
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
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_CONFIG,
        SIZE_1_BYTE,
        &readBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
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
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_GYRO_CONFIG,
        SIZE_1_BYTE,
        &readBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
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
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_ACCEL_CONFIG,
        SIZE_1_BYTE,
        &readBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
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
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_SMPRT_DIV,
        SIZE_1_BYTE,
        &readBuff,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
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
    uint8_t measureUpper = 0;
    uint8_t measureLower = 0;
    //upper portion of accel 
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        address,
        SIZE_1_BYTE,
        &measureUpper,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    //lower portion
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        address + 1,
        SIZE_1_BYTE,
        &measureLower,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    int16_t combined = (int16_t)(measureUpper << 8) | measureLower;
    float scaled = (float)combined / scaler;
    return scaled;
}

int16_t read_raw_accel_axis(uint8_t address)
{
    uint8_t measureUpper = 0;
    uint8_t measureLower = 0;
    //upper portion of accel 
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        address,
        SIZE_1_BYTE,
        &measureUpper,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    //lower portion
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        address + 1,
        SIZE_1_BYTE,
        &measureLower,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    int16_t combined = (int16_t)(measureUpper << 8) | measureLower;
    return combined;
}

float read_gyro_axis(uint8_t address, uint16_t scaler)
{
    uint8_t measureUpper = 0;
    uint8_t measureLower = 0;
    //upper portion of gyeo x
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        address,
        SIZE_1_BYTE,
        &measureUpper,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    //lower portion
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        address + 1,
        SIZE_1_BYTE,
        &measureLower,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    int16_t combined = (int16_t)(measureUpper << 8) | measureLower;
    float scaled = (float)combined / scaler;
    return scaled;
}

int16_t read_raw_gyro_axis(uint8_t address)
{
    uint8_t measureUpper = 0;
    uint8_t measureLower = 0;
    //upper portion of gyeo x
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        address,
        SIZE_1_BYTE,
        &measureUpper,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    //lower portion
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        address + 1,
        SIZE_1_BYTE,
        &measureLower,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

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
    uint8_t gyroFS = 0;

    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_GYRO_CONFIG,
        SIZE_1_BYTE,
        &gyroFS,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
    gyroFS &= GYRO_FS_SEL_MASK; //keep only the FS_SEL setting

    //set gyro to 250 dps for test
    MPU6050_REG_WRITE(REG_GYRO_CONFIG, GYRO_FS_SEL_250_DPS);

    //wait
    HAL_Delay(250);

    //get gyro's output with self test disabled
    int16_t TD[3]; //3 axis
    TD[0] = read_raw_gyro_axis(REG_GYRO_X_MEASURE_1);
    TD[1] = read_raw_gyro_axis(REG_GYRO_Y_MEASURE_1);
    TD[2] = read_raw_gyro_axis(REG_GYRO_Z_MEASURE_1);

    //enable self test, and datasheet requires gyro set to 250 DPS
    MPU6050_REG_WRITE(
        REG_GYRO_CONFIG, 
        GYRO_FS_SEL_250_DPS | GYRO_XG_ST | GYRO_YG_ST | GYRO_ZG_ST
    );

    //wait
    HAL_Delay(250);
    
    //get gyro's output with self test enabled
    int16_t TE[3]; //3 axis
    TE[0] = read_raw_gyro_axis(REG_GYRO_X_MEASURE_1);
    TE[1] = read_raw_gyro_axis(REG_GYRO_Y_MEASURE_1);
    TE[2] = read_raw_gyro_axis(REG_GYRO_Z_MEASURE_1);

    //calculate the value of STR from the datasheet. This is
    //different from reading the SELF_TEST (GTest) registers below
    int16_t selfTestResponse[3];
    selfTestResponse[0] = TE[0] - TD[0];
    selfTestResponse[1] = TE[1] - TD[1];
    selfTestResponse[2] = TE[2] - TD[2];
    

    //read self test registers
    uint8_t GTest[3];

    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_SELF_TEST_X,
        SIZE_1_BYTE,
        &GTest[0],
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_SELF_TEST_Y,
        SIZE_1_BYTE,
        &GTest[1],
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_SELF_TEST_Z,
        SIZE_1_BYTE,
        &GTest[2],
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

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
    MPU6050_REG_WRITE(REG_GYRO_CONFIG, gyroFS);

    
    return FACTORY_TEST_PASS;
}

FACTORY_TEST_RESULT accel_self_test(void)
{
    //save old accel full scale range
    uint8_t accelFS = 0;

    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_ACCEL_CONFIG,
        SIZE_1_BYTE,
        &accelFS,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
    accelFS &= ACCEL_FS_SEL_MASK; //keep only the FS_SEL setting

    //set accel to 8g for test
    MPU6050_REG_WRITE(REG_ACCEL_CONFIG, ACCEL_FS_8G);

    //wait
    HAL_Delay(250);

    //get accels's output with self test disabled
    int16_t TD[3]; //3 axis
    TD[0] = read_raw_accel_axis(REG_ACCEL_X_MEASURE_1);
    TD[1] = read_raw_accel_axis(REG_ACCEL_Y_MEASURE_1);
    TD[2] = read_raw_accel_axis(REG_ACCEL_Z_MEASURE_1);

    //enable self test, and datasheet requires accel set to 8g
    MPU6050_REG_WRITE(
        REG_ACCEL_CONFIG, 
        ACCEL_FS_8G | ACCEL_XA_ST | ACCEL_YA_ST | ACCEL_ZA_ST
    );

    //wait
    HAL_Delay(250);
    
    //get accels's output with self test enabled
    int16_t TE[3]; //3 axis
    TE[0] = read_raw_accel_axis(REG_ACCEL_X_MEASURE_1);
    TE[1] = read_raw_accel_axis(REG_ACCEL_Y_MEASURE_1);
    TE[2] = read_raw_accel_axis(REG_ACCEL_Z_MEASURE_1);

    //calculate the value of STR from the datasheet. This is
    //different from reading the SELF_TEST (ATest) registers below
    int16_t selfTestResponse[3];
    selfTestResponse[0] = TE[0] - TD[0];
    selfTestResponse[1] = TE[1] - TD[1];
    selfTestResponse[2] = TE[2] - TD[2];
    

    //read self test registers, each axis is 5 bits, split across two registers
    uint8_t ATestUpper[3]; //more significant portion
    uint8_t SELF_TEST_A; //less significant portion

    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_SELF_TEST_X,
        SIZE_1_BYTE,
        &ATestUpper[0],
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_SELF_TEST_Y,
        SIZE_1_BYTE,
        &ATestUpper[1],
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_SELF_TEST_Z,
        SIZE_1_BYTE,
        &ATestUpper[2],
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

    //lower
    HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        REG_SELF_TEST_A,
        SIZE_1_BYTE,
        &SELF_TEST_A,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );

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
    MPU6050_REG_WRITE(REG_ACCEL_CONFIG, accelFS);

    
    return FACTORY_TEST_PASS;
}

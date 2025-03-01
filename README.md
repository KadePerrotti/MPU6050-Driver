# Platform Independent MPU6050 Driver

## Overview
This is a driver for the MPU6050, a 6-axis motion tracking device that includes a 3-axis gyroscope and a 3-axis accelerometer. See the [datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf) for more details. 

This driver provides functions for initializing the device, reading sensor data, and running self-tests. It does not aim to be a feature complete driver, like the one in [i2cdevlib](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050). Please use a more tested and complete driver for serious projects, this is just a learning opportunity for me. 



## Features
- Initialization and configuration of the MPU6050
- Register read and write functionality
- Reading accelerometer and gyroscope values
- Self-test functionality for gyroscope, accelerometer, and fifo
- FIFO buffer management

## Files
- *MPU6050.c / .h*: Typedefs for required function implementations, and functions to init, read axes, and complete self test procedures.
- *REG_OPTIONS.h*: Register addresses, common register values, and masks. Not a complete map by any means.
- *TEST_FUNCTIONS.c / .h*: Functions that test operations on the MPU6050 such as reading axes or the fifo buffer. They give examples for how this driver could be used as part of a larger application. **Not required to use the driver**.

## Example Projects
See the [example projects]() for demonstrations on various platforms.

## Requirements
This driver is designed to be platform-independent and requires the user to provide implementations for low-level register read, write, and timing functions.

### Required Function Implementations
Users must implement functions that conform to the following types:
```c
typedef int MPU6050_REG_READ_TYPE(uint16_t address, uint8_t* data);
typedef int MPU6050_BURST_READ_TYPE(uint16_t address, uint8_t* data, uint16_t length);
typedef int MPU6050_REG_WRITE_TYPE(uint16_t address, uint8_t data);
typedef void DELAY_MS_TYPE(uint32_t ms);
typedef uint32_t TIME_MS_TYPE(void);
```

For example, on an Arduino platform:
```c++
#include <Wire.h>
#include "src/MPU6050-Driver/MPU6050.h"

/**
 * @brief Writes a single byte to the specified MPU6050 register
 * using arduino's i2c library
 * @param regAddress the address of the register to write
 * @param data the data to write to the register
 * @return status of the transmission
 */
int arduino_reg_write_mpu6050(uint16_t regAddress, uint8_t data)
{
  Wire.beginTransmission(MPU_6050_ADDR);
  Wire.write(regAddress);
  Wire.write(data);
  uint8_t status = Wire.endTransmission();
  return status;
}

void setup() 
{
    // initialize i2c
    Wire.begin();

    // Assign function pointer to the Arduino implementation of 
    // a single-byte I2C write
    MPU6050_REG_WRITE_TYPE* writeReg = arduino_reg_write_mpu6050;

    // Arduino already has a millisecond delay function that matches  
    // our requirements, so we don't need to implement another
    DELAY_MS_TYPE* delayMs = delay; 
    
    // Pass our custom functions to init
    init_mpu6050(writeReg, delayMs);
}
```

## API Reference

### Initialization
```c
uint16_t init_mpu6050(MPU6050_REG_WRITE_TYPE writeReg, DELAY_MS_TYPE delay);
```
Initializes the MPU6050 with default settings.

### Sensor Readings
#### Accelerometer
```c
float read_accel_axis(uint8_t address, uint16_t scaler, MPU6050_REG_READ_TYPE readReg);
```
Reads and scales an accelerometer axis value in G (9.8m/s^2).

#### Gyroscope
```c
float read_gyro_axis(uint8_t address, uint16_t scaler, MPU6050_REG_READ_TYPE readReg);
```
Reads and scales a gyroscope axis value in degrees per second (DPS).

#### Raw Data
```c
int16_t read_raw_axis(uint8_t address, MPU6050_REG_READ_TYPE readReg);
```
Reads the raw sensor data from a specified axis.

### Self-Test Functions
```c
FACTORY_TEST_RESULTS gyro_self_test(MPU6050_REG_READ_TYPE readReg, MPU6050_REG_WRITE_TYPE writeReg, DELAY_MS_TYPE delay);
FACTORY_TEST_RESULTS accel_self_test(MPU6050_REG_READ_TYPE readReg, MPU6050_REG_WRITE_TYPE writeReg, DELAY_MS_TYPE delay);
```
Completes the self test procedures described in the datasheet for the gyroscope and accelerometer to verify functionality, and returns a struct containing test results.

### FIFO Buffer Management
```c
uint16_t read_fifo_count(MPU6050_REG_READ_TYPE readReg);
```
Reads the number of bytes currently stored in the MPU6050's FIFO buffer.

## Basic STM32 Example
See the example repos for more in depth usage
```c
#include "Drivers/MPU6050-Driver/REG_OPTIONS.h"
#include "Drivers/MPU6050-Driver/MPU6050.h"


/**
 * @brief Writes a single byte to the specified MPU6050 register
 * using ST's i2c HAL
 * @param regAddress the address of the register to write
 * @param data the data to write to the register
 * @return status of the transmission
 */
int MPU6050_REG_WRITE_STM32(uint16_t regAddr, uint8_t regValue)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        &hi2c1,
        MPU_6050_HAL_I2C_ADDR,
        regAddr,
        I2C_MEMADD_SIZE_8BIT,
        &regValue,
        SIZE_1_BYTE,
        HAL_I2C_TIMEOUT
    );
    return status;
}

/**
 * @brief Reads a single byte from the specified MPU6050 register
 * using ST's i2c HAL
 * @param regAddress the address of the register to read
 * @param data memory to read data into
 * @return status of the transmission
 */
int MPU6050_REG_READ_STM32(uint16_t regAddr, uint8_t* valAddr)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1, 
        MPU_6050_HAL_I2C_ADDR,
        regAddr,
        I2C_MEMADD_SIZE_8BIT,
        valAddr,
        I2C_MEMADD_SIZE_8BIT,
        HAL_I2C_TIMEOUT
    );
    return status;
}

int main(void)
{
    // Assign function pointers to the STM32 implementations of 
    // reads, writes, and timing.
    MPU6050_REG_WRITE_TYPE* writeReg = MPU6050_REG_WRITE_STM32;
    MPU6050_REG_READ_TYPE* readReg = MPU6050_REG_READ_STM32;
    DELAY_MS_TYPE* delay = HAL_Delay;
    TIME_MS_TYPE* getTime = HAL_GetTick;

    //basic initialization
    init_mpu6050(writeReg, delay);

    //configure important accelerometer and gyro settings
  
    //setup the low pass filter. 
    writeReg(REG_CONFIG, DLPF_CFG_6 | EXT_SYNC_OFF);

    //select Gyroscope's full scale range
    writeReg(REG_GYRO_CONFIG, GYRO_FS_SEL_250_DPS);   
    
    //select Accelerometer's full scale range
    writeReg(REG_ACCEL_CONFIG, ACCEL_FS_SEL_2G);

    //setup the sample rate divider
    writeReg(REG_SMPRT_DIV, SAMPLE_RATE_50Hz);

    //read all 6 axis
    float samplePeriodMS = 20; //sample rate is 50Hz, so 20ms period for each sample
    while(true)
    {
        uint32_t startRead = getTime();
        float accelX = read_accel_axis(REG_ACCEL_X_MEASURE_1, ACCEL_FS_2_DIV, readReg);
        
        float accelY = read_accel_axis(REG_ACCEL_Y_MEASURE_1, ACCEL_FS_2_DIV, readReg);
        
        float accelZ = read_accel_axis(REG_ACCEL_Z_MEASURE_1, ACCEL_FS_2_DIV, readReg);

        float gyroX = read_gyro_axis(REG_GYRO_X_MEASURE_1, GYRO_FS_250_DIV, readReg);

        float gyroY = read_gyro_axis(REG_GYRO_Y_MEASURE_1, GYRO_FS_250_DIV, readReg);

        float gyroZ = read_gyro_axis(REG_GYRO_Z_MEASURE_1, GYRO_FS_250_DIV, readReg);

        int endRead = getTime();
        int totalTime = endRead - startRead; //ms
        delay(samplePeriodMs - totalTime); 
    }
}
```


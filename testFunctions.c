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
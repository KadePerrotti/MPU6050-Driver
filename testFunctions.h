/*
 * testFunctions.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Kade
 */

#ifndef INC_TESTFUNCTIONS_H_
#define INC_TESTFUNCTIONS_H_

#include "MPU6050.h"

typedef struct 
{
    //Contained in REG_CONFIG
    uint8_t fsync;
    uint8_t dlpf;

    //GYRO_CONFIG
    uint8_t gyro_sel;

    //ACCEL_CONFIG
    uint8_t fs;

    //SMPRT_DIV
    uint8_t rate_div;
} SETUP_REGISTERS;

/**
 * Self-test function that reads configuration registers
 * and masks the desired variables. Returns a struct containing the
 * them.
 * 
 * Checks: REG_CONFIG, REG_GYRO_CONFIG, REG_ACCEL_CONFIG,
 * REG_SMPRT_DIV
 */
SETUP_REGISTERS read_setup_registers(MPU6050_REG_READ_TYPE readReg);

#endif /* INC_TESTFUNCTIONS_H_ */

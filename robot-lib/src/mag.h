/// @file mag.h
/// MEthods for accessing mag data from
/// lis3mdl sensor

#pragma once
#include <hardware/i2c.h>

// 100KHz I2C Bus0 baudrate
#define _MAG_I2C0_BAUDRATE 400000
#define _MAG_ADDR 0b0011110

#define _MAG_REG_WHO_AM_I 0x0F
#define _MAG_REG_CTRL1 0x20
#define _MAG_REG_CTRL2 0x21
#define _MAG_REG_CTRL3 0x22
#define _MAG_REG_CTRL4 0x23
#define _MAG_REG_CTRL5 0x24
#define _MAG_STATUS_REG 0x27
#define _MAG_OUTX_L 0x28

typedef struct _mag_inst {
    uint32_t set_baud
    bool status;
} mag_inst_t;

typedef struct _axes_data {
    float x;
    float y;
    float z;
} axes_data_t;

/// @brief 
/// initialize the i2c instance and set pins
uint mag_init(mag_inst_t* mag_inst);

/// @brief fills the provided memory space
/// with the mag data in gauss
/// for each axis.
/// @param mag_data 
uint mag_read(imu_inst_t* mag_inst, axes_data_t* mag_data);

// private helpers
/// @brief sets configuration registers
void _mag_set();

/// @brief write to imu reset register on i2c
void _mag_reset();

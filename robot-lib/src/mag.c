/// @file mag.c
/// contains implementations for the magnetometer
#include <mag.h>
#include <hardware/i2c.h>
#include <hardware/gpio.h>

// Output Data Rate
const uint8_t odr = 0b101; // 20Hz

// Measurement Range
const uint8_t fs = 0b00; // +- 4gauss

// Sensitivity Multiplier
const float sen_mag = 6842; // gauss/LSB

uint mag_init(mag_inst_t* mag_inst) {
    uint8_t w_cmd;
    uint8_t r_buf;

    if (mag_inst->status) {
        return 0;
    }
    
    // i2c peripheral instance 0
    // initialize instance and record baudrate
    mag_inst->set_baud = i2c_init(i2c0, _MAG_I2C0_BAUDRATE);
    
    // sda: data line
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_pull_up(4);
   
    // scl: clk line 
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(5);

    // read from WHO AM I register
    w_cmd = _MAG_REG_WHO_AM_I;
    // keep bus control
    i2c_write_blocking(i2c0, _MAG_ADDR, &w_cmd, 1, true);
    i2c_read_blocking(i2c0, _MAG_ADDR, &r_buf, 1, false);
    mag_inst->status = (r_buf == 0x6C);
    
    if (!mag_inst->status) {
        return -1;
    }

    _mag_reset();
    _mag_set();
    return 0;    
}

void _mag_reset() {
    uint8_t cmd_buf[2] = {_MAG_REG_CTRL3_C, 0x00};
    // read and store CTRL3
    i2c_write_blocking(i2c0, _MAG_ADDR, &(cmd_buf[0]), 1, true);
    i2c_read_blocking(i2c0, _MAG_ADDR, &(cmd_buf[1]), 1, false);
    // mask and write
    // write to _CTRL3_C.BOOT (0x80)
    cmd_buf[1] |= 0x80;
    i2c_write_blocking(i2c0, _MAG_ADDR, cmd_buf, 2, false);
    // blocking read until _CTRL3_C.BOOT flag resets
    do {
        i2c_write_blocking(i2c0, _MAG_ADDR, &(cmd_buf[0]), 1, true);
        i2c_read_blocking(i2c0, _MAG_ADDR, &(cmd_buf[1]), 1, false);
    } while(cmd_buf[1] & 0x80);

    /// TODO: sleep and check at end or poll check
    /// use an interrupt to fire when reset complete
    /// callback update the imu_ins->status

    // mask and write
    // write to _CTRL3_C.SW_RESET (0x01)
    cmd_buf[1] |= 0x01;
    i2c_write_blocking(i2c0, _MAG_ADDR, cmd_buf, 2, false);
    // blocking read until _CTRL3_C.SW_RESET flag resets
    do {
        i2c_write_blocking(i2c0, _MAG_ADDR, &(cmd_buf[0]), 1, true);
        i2c_read_blocking(i2c0, _MAG_ADDR, &(cmd_buf[1]), 1, false);
    } while(cmd_buf[1] & 0x01);
}

void _mag_set() {
    // imu general configuration
    uint8_t cmd_buf[2] = {_MAG_REG_CTRL3_C, 0x00};
    // read and store CTRL3
    i2c_write_blocking(i2c0, _MAG_ADDR, &(cmd_buf[0]), 1, true);
    i2c_read_blocking(i2c0, _MAG_ADDR, &(cmd_buf[1]), 1, false);
    // write to _CTRL3_C.BDU (0x40)
    cmd_buf[1] |= 0x40;
    i2c_write_blocking(i2c0, _MAG_ADDR, cmd_buf, 2, false);
    
    uint8_t cmd_mag_buf[2] = {_MAG_REG_CTRL1_XL, 0x00};
    
    // write to CTRL1_XL
    cmd_acc_buf[1] = (odr_acc << 4 | fs_acc << 2) | 0b00;
    i2c_write_blocking(i2c0, _IMU_ADDR, cmd_acc_buf, 2, false);
    
    // write to CTRL2_G
    cmd_gyro_buf[1] = (odr_gyro << 4 | fs_gyro << 1) | 0b0;
    i2c_write_blocking(i2c0, _IMU_ADDR, cmd_gyro_buf, 2, false);
}

uint mag_read(imu_inst_t* imu_inst, axes_data_t* mag_data) {
    if (!imu_inst->status) {
        return -1;
    }  
    uint8_t reg = _MAG_OUTX_L;
    uint8_t buf[6];
    int16_t raw[3];
    // read from the register 
    i2c_write_blocking(i2c0, _MAG_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, _MAG_ADDR, buf, 6, false);
    /// TODO: might be other way
    for (int i = 0; i < 3; i++) {
        raw[i] = (buf[(i * 2) + 1] << 8 | buf[i * 2]);
    }
    mag_data->x = (raw[0] * sen_mag) / 1000.0;
    mag_data->y = (raw[1] * sen_mag) / 1000.0;
    mag_data->z = (raw[2] * sen_mag) / 1000.0;
    return 0;
}
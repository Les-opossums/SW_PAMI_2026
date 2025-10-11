// mpu6500.c

#include "mpu6500.h"
#include <stdio.h>

// Static helper functions for I2C communication
static void i2c_write_reg_byte(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(i2c, addr, data, 2, false);
}

static void i2c_read_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    i2c_write_blocking(i2c, addr, &reg, 1, true); // true to keep master control
    i2c_read_blocking(i2c, addr, buf, len, false);
}

static void i2c_write_reg_u16_be(i2c_inst_t *i2c, uint8_t addr, uint8_t reg_h, int16_t value) {
    uint8_t data[3] = {reg_h, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    i2c_write_blocking(i2c, addr, data, 3, false);
}

// Public functions
void mpu6500_init(mpu6500_t *mpu, i2c_inst_t *i2c, uint8_t address) {
    mpu->i2c_instance = i2c;
    mpu->i2c_address = address;

    // Initialize offsets to zero
    mpu->gyro_offset.x = 0;
    mpu->gyro_offset.y = 0;
    mpu->gyro_offset.z = 0;

    mpu->accel_offset.x = 0;
    mpu->accel_offset.y = 0;
    mpu->accel_offset.z = 0;
}

bool mpu6500_begin(mpu6500_t *mpu) {
    // Reset the device to restore default settings
    i2c_write_reg_byte(mpu->i2c_instance, mpu->i2c_address, MPU6500_REG_PWR_MGMT_1, 0x80);
    sleep_ms(100);

    // Wake up device and set clock source to gyroscope Z-axis reference
    i2c_write_reg_byte(mpu->i2c_instance, mpu->i2c_address, MPU6500_REG_PWR_MGMT_1, 0x03);
    sleep_ms(10);
    
    // Check WHO_AM_I register
    uint8_t who_am_i;
    i2c_read_reg(mpu->i2c_instance, mpu->i2c_address, MPU6500_REG_WHO_AM_I, &who_am_i, 1);
    
    if (who_am_i != MPU6500_WHO_AM_I_VAL) {
        printf("Error: MPU-6500 not found at address 0x%02X. WHO_AM_I: 0x%02X\n", mpu->i2c_address, who_am_i);
        return false;
    }
    
    // Set default ranges
    mpu6500_set_accel_range(mpu, ACCEL_FS_4G);
    mpu6500_set_gyro_scale(mpu, GYRO_FS_500_DPS);

    // Set Digital Low Pass Filter (DLPF)
    // A setting of 3 yields a bandwidth of ~41Hz for both accel and gyro
    i2c_write_reg_byte(mpu->i2c_instance, mpu->i2c_address, MPU6500_REG_CONFIG, 0x03);

    printf("MPU-6500 successfully initialized.\n");
    return true;
}

void mpu6500_set_gyro_scale(mpu6500_t *mpu, mpu6500_gyro_fs_t scale) {
    uint8_t value = (scale << 3);
    i2c_write_reg_byte(mpu->i2c_instance, mpu->i2c_address, MPU6500_REG_GYRO_CONFIG, value);

    // Update the scaler based on the selected range
    switch(scale) {
        case GYRO_FS_250_DPS:  mpu->gyro_scaler = 131.0f; break;
        case GYRO_FS_500_DPS:  mpu->gyro_scaler = 65.5f;  break;
        case GYRO_FS_1000_DPS: mpu->gyro_scaler = 32.8f;  break;
        case GYRO_FS_2000_DPS: mpu->gyro_scaler = 16.4f;  break;
    }
}

void mpu6500_set_accel_range(mpu6500_t *mpu, mpu6500_accel_fs_t range) {
    uint8_t value = (range << 3);
    i2c_write_reg_byte(mpu->i2c_instance, mpu->i2c_address, MPU6500_REG_ACCEL_CONFIG, value);

    // Update the scaler based on the selected range
    switch(range) {
        case ACCEL_FS_2G:  mpu->accel_scaler = 16384.0f; break;
        case ACCEL_FS_4G:  mpu->accel_scaler = 8192.0f;  break;
        case ACCEL_FS_8G:  mpu->accel_scaler = 4096.0f;  break;
        case ACCEL_FS_16G: mpu->accel_scaler = 2048.0f;  break;
    }
}

void mpu6500_read_raw(mpu6500_t *mpu) {
    uint8_t buffer[14];
    // Read 14 bytes starting from the accelerometer data registers
    i2c_read_reg(mpu->i2c_instance, mpu->i2c_address, MPU6500_REG_ACCEL_XOUT_H, buffer, 14);

    // Combine high and low bytes for each sensor value
    mpu->raw_accel.x = (int16_t)(buffer[0] << 8 | buffer[1]);
    mpu->raw_accel.y = (int16_t)(buffer[2] << 8 | buffer[3]);
    mpu->raw_accel.z = (int16_t)(buffer[4] << 8 | buffer[5]);

    mpu->raw_temp = (int16_t)(buffer[6] << 8 | buffer[7]);

    mpu->raw_gyro.x = (int16_t)(buffer[8] << 8 | buffer[9]);
    mpu->raw_gyro.y = (int16_t)(buffer[10] << 8 | buffer[11]);
    mpu->raw_gyro.z = (int16_t)(buffer[12] << 8 | buffer[13]);
}

void mpu6500_get_accel_g(mpu6500_t *mpu, mpu6500_float_data_t *data) {
    data->x = (float)(mpu->raw_accel.x - mpu->accel_offset.x) / mpu->accel_scaler;
    data->y = (float)(mpu->raw_accel.y - mpu->accel_offset.y) / mpu->accel_scaler;
    data->z = (float)(mpu->raw_accel.z - mpu->accel_offset.z) / mpu->accel_scaler;
}

void mpu6500_get_gyro_dps(mpu6500_t *mpu, mpu6500_float_data_t *data) {
    data->x = (float)(mpu->raw_gyro.x - mpu->gyro_offset.x) / mpu->gyro_scaler;
    data->y = (float)(mpu->raw_gyro.y - mpu->gyro_offset.y) / mpu->gyro_scaler;
    data->z = (float)(mpu->raw_gyro.z - mpu->gyro_offset.z) / mpu->gyro_scaler;
}

float mpu6500_get_temp_c(mpu6500_t *mpu) {
    // MPU-6500 temperature formula from datasheet:
    // Temp_C = ((Temp_out - RoomTemp_Offset) / Temp_Sensitivity) + 21.0
    // We use a simplified version which is very close.
    return ((float)mpu->raw_temp / 333.87f) + 21.0f;
}

void mpu6500_calibrate_gyro(mpu6500_t *mpu, uint16_t num_samples) {
    printf("Starting gyroscope calibration. Keep the sensor stationary...\n");
    // Use long to prevent overflow
    long gyro_x_sum = 0;
    long gyro_y_sum = 0;
    long gyro_z_sum = 0;

    // Take a number of readings and sum them
    for (int i = 0; i < num_samples; i++) {
        mpu6500_read_raw(mpu);
        gyro_x_sum += mpu->raw_gyro.x;
        gyro_y_sum += mpu->raw_gyro.y;
        gyro_z_sum += mpu->raw_gyro.z;
        sleep_ms(5); // Small delay between readings
    }

    // Calculate the average bias
    mpu->gyro_offset.x = (int16_t)(gyro_x_sum / num_samples);
    mpu->gyro_offset.y = (int16_t)(gyro_y_sum / num_samples);
    mpu->gyro_offset.z = (int16_t)(gyro_z_sum / num_samples);
    printf("Calibration complete. Raw bias: X=%d, Y=%d, Z=%d\n", mpu->gyro_offset.x, mpu->gyro_offset.y, mpu->gyro_offset.z);
}

void mpu6500_calibrate_accel(mpu6500_t *mpu, uint16_t num_samples) {
    printf("Starting accelerometer calibration. Keep the sensor stationary...\n");
    // Use long to prevent overflow
    long accel_x_sum = 0;
    long accel_y_sum = 0;
    long accel_z_sum = 0;

    // Take a number of readings and sum them
    for (int i = 0; i < num_samples; i++) {
        mpu6500_read_raw(mpu);
        accel_x_sum += mpu->raw_accel.x;
        accel_y_sum += mpu->raw_accel.y;
        accel_z_sum += mpu->raw_accel.z;
        sleep_ms(5); // Small delay between readings
    }

    // Calculate the average bias
    mpu->accel_offset.x = (int16_t)(accel_x_sum / num_samples);
    mpu->accel_offset.y = (int16_t)(accel_y_sum / num_samples);
    mpu->accel_offset.z = (int16_t)(accel_z_sum / num_samples);
    printf("Calibration complete. Raw bias: X=%d, Y=%d, Z=%d\n", mpu->accel_offset.x, mpu->accel_offset.y, mpu->accel_offset.z);
}

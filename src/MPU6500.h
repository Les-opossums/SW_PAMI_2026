// mpu6500.h

#ifndef MPU6500_H
#define MPU6500_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Default I2C address for the MPU-6500
#define MPU6500_I2C_ADDR 0x68

// WHO_AM_I register value for MPU-6500
#define MPU6500_WHO_AM_I_VAL 0x70

// Standard gravity constant
#define GRAVITY_G 9.80665f

// MPU-6500 Register Map
#define MPU6500_REG_ACCEL_XOUT_H    0x3B
#define MPU6500_REG_ACCEL_XOUT_L    0x3C
#define MPU6500_REG_ACCEL_YOUT_H    0x3D
#define MPU6500_REG_ACCEL_YOUT_L    0x3E
#define MPU6500_REG_ACCEL_ZOUT_H    0x3F
#define MPU6500_REG_ACCEL_ZOUT_L    0x40
#define MPU6500_REG_TEMP_OUT_H      0x41
#define MPU6500_REG_TEMP_OUT_L      0x42
#define MPU6500_REG_GYRO_XOUT_H     0x43
#define MPU6500_REG_GYRO_XOUT_L     0x44
#define MPU6500_REG_GYRO_YOUT_H     0x45
#define MPU6500_REG_GYRO_YOUT_L     0x46
#define MPU6500_REG_GYRO_ZOUT_H     0x47
#define MPU6500_REG_GYRO_ZOUT_L     0x48

#define MPU6500_REG_CONFIG          0x1A
#define MPU6500_REG_GYRO_CONFIG     0x1B
#define MPU6500_REG_ACCEL_CONFIG    0x1C
#define MPU6500_REG_ACCEL_CONFIG_2  0x1D
#define MPU6500_REG_PWR_MGMT_1      0x6B
#define MPU6500_REG_WHO_AM_I        0x75

// Enum for Gyroscope Full Scale Range
typedef enum {
    GYRO_FS_250_DPS = 0,
    GYRO_FS_500_DPS,
    GYRO_FS_1000_DPS,
    GYRO_FS_2000_DPS
} mpu6500_gyro_fs_t;

// Enum for Accelerometer Full Scale Range
typedef enum {
    ACCEL_FS_2G = 0,
    ACCEL_FS_4G,
    ACCEL_FS_8G,
    ACCEL_FS_16G
} mpu6500_accel_fs_t;

// Struct to hold raw sensor data (16-bit integers)
typedef struct {
    int16_t x, y, z;
} mpu6500_raw_data_t;

// Struct to hold processed sensor data (floats)
typedef struct {
    float x, y, z;
} mpu6500_float_data_t;

// Main device structure
typedef struct {
    i2c_inst_t *i2c_instance;
    uint8_t i2c_address;

    mpu6500_raw_data_t raw_accel;
    mpu6500_raw_data_t raw_gyro;
    int16_t raw_temp;

    float accel_scaler;
    float gyro_scaler;
} mpu6500_t;

// Function Prototypes
void mpu6500_init(mpu6500_t *mpu, i2c_inst_t *i2c, uint8_t address);
bool mpu6500_begin(mpu6500_t *mpu);
void mpu6500_set_gyro_scale(mpu6500_t *mpu, mpu6500_gyro_fs_t scale);
void mpu6500_set_accel_range(mpu6500_t *mpu, mpu6500_accel_fs_t range);
void mpu6500_read_raw(mpu6500_t *mpu);

// Data conversion functions
void mpu6500_get_accel_g(mpu6500_t *mpu, mpu6500_float_data_t *data);
void mpu6500_get_gyro_dps(mpu6500_t *mpu, mpu6500_float_data_t *data);
float mpu6500_get_temp_c(mpu6500_t *mpu);

#endif // MPU6500_H
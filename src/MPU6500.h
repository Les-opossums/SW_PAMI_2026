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

// Self-test registers
#define MPU6500_REG_SELF_TEST_X_GYRO 0x00
#define MPU6500_REG_SELF_TEST_Y_GYRO 0x01
#define MPU6500_REG_SELF_TEST_Z_GYRO 0x02
#define MPU6500_REG_SELF_TEST_X_ACCEL 0x0D
#define MPU6500_REG_SELF_TEST_Y_ACCEL 0x0E
#define MPU6500_REG_SELF_TEST_Z_ACCEL 0x0F

// Offset registers
#define MPU6500_REG_XG_OFFSET_H  0x13
#define MPU6500_REG_XG_OFFSET_L  0x14
#define MPU6500_REG_YG_OFFSET_H  0x15
#define MPU6500_REG_YG_OFFSET_L  0x16
#define MPU6500_REG_ZG_OFFSET_H  0x17
#define MPU6500_REG_ZG_OFFSET_L  0x18

// Data registers for accelerometer
#define MPU6500_REG_ACCEL_XOUT_H    0x3B
#define MPU6500_REG_ACCEL_XOUT_L    0x3C
#define MPU6500_REG_ACCEL_YOUT_H    0x3D
#define MPU6500_REG_ACCEL_YOUT_L    0x3E
#define MPU6500_REG_ACCEL_ZOUT_H    0x3F
#define MPU6500_REG_ACCEL_ZOUT_L    0x40

// Data registers for temperature
#define MPU6500_REG_TEMP_OUT_H      0x41
#define MPU6500_REG_TEMP_OUT_L      0x42

// Data registers for gyroscope
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



#define VEL_DECAY           0.95f      // velocity damping factor per update when stationary
#define ACC_THRESHOLD       0.05f      // m/s²
#define BIAS_FILTER_ALPHA   0.005f     // low-pass for bias estimation
#define ODO_POS_EVERY_SPEED 5        


// Gravity constant in m/s^2
#define G_ACCEL 9.81f

// Complementary filter coefficient (alpha).
// 0.98 is a good starting point, trusting the gyro 98% and the accelerometer 2%.
#define COMPLEMENTARY_FILTER_ALPHA 0.98f

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

// Struct to hold offset data (16-bit integers)
typedef struct {
    int16_t x, y, z;
} mpu6500_offset_data_t;


// Struct to hold processed sensor data (floats)
typedef struct {
    float x, y, z;
} mpu6500_float_data_t;

typedef struct {
    float x;      // position in world frame (m)
    float y;
    float theta;  // yaw angle (rad)

    float vx;     // velocity in body frame (m/s)
    float vy;
    float wz;     // angular velocity in rad/s

    float cumul_vx; // cumulative velocity
    float cumul_vy;
    float cumul_wz;

    float accel_bias_x;
    float accel_bias_y;
    float gyro_bias_z;

    float pitch; // estimated pitch angle (rad)
    float roll;  // estimated roll angle (rad)

    float g_est_x; // estimated gravity vector
    float g_est_y;
    float g_est_z;

    // inside mpu6500_odometry_t
    float q0, q1, q2, q3;      // Madgwick quaternion (unit)
    float madgwick_beta;       // filter gain

    uint64_t speed_last_time_us;
    uint64_t position_last_time_us;

    uint32_t speed_update_counter;
} mpu6500_odometry_t;

// Main device structure
typedef struct {
    i2c_inst_t *i2c_instance;
    uint8_t i2c_address;

    mpu6500_raw_data_t raw_accel;
    mpu6500_raw_data_t raw_gyro;
    int16_t raw_temp;

    mpu6500_offset_data_t gyro_offset;
    mpu6500_offset_data_t accel_offset;

    float accel_scaler;
    float gyro_scaler;

    mpu6500_odometry_t odom;
} mpu6500_t;



extern mpu6500_t mpu6500;


/**
 * @brief Initializes the MPU-6500 sensor.
 *
 * @param mpu Pointer to the mpu6500_t struct.
 * @param i2c Pointer to the I2C instance.
 * @param address I2C address of the MPU-6500.
 */
void mpu6500_init(mpu6500_t *mpu, i2c_inst_t *i2c, uint8_t address);

/** 
 * @brief Initializes communication with the MPU-6500 and configures default settings.
 */
bool mpu6500_begin(mpu6500_t *mpu);

/**
 * @brief Sets the gyroscope full-scale range.
 *
 * @param mpu Pointer to the mpu6500_t struct.
 * @param scale Desired gyroscope full-scale range.
 */
void mpu6500_set_gyro_scale(mpu6500_t *mpu, mpu6500_gyro_fs_t scale);

/**
 * @brief Sets the accelerometer full-scale range.
 *
 * @param mpu Pointer to the mpu6500_t struct.
 * @param range Desired accelerometer full-scale range.
 */
void mpu6500_set_accel_range(mpu6500_t *mpu, mpu6500_accel_fs_t range);

/**
 * @brief Reads raw sensor data from the MPU-6500.
 *
 * @param mpu Pointer to the mpu6500_t struct.
 */
void mpu6500_read_raw(mpu6500_t *mpu);

/**
 * @brief Converts raw accelerometer data to g's.
 *
 * @param mpu Pointer to the mpu6500_t struct.
 * @param data Pointer to mpu6500_float_data_t struct to store converted data.
 */
void mpu6500_get_accel_g(mpu6500_t *mpu, mpu6500_float_data_t *data);

/**
 * @brief Converts raw gyroscope data to degrees per second (dps).
 *
 * @param mpu Pointer to the mpu6500_t struct.
 * @param data Pointer to mpu6500_float_data_t struct to store converted data.
 */
void mpu6500_get_gyro_dps(mpu6500_t *mpu, mpu6500_float_data_t *data);

/**
 * @brief Converts raw temperature data to degrees Celsius.
 *
 * @param mpu Pointer to the mpu6500_t struct.
 * @return Temperature in degrees Celsius.
 */
float mpu6500_get_temp_c(mpu6500_t *mpu);

/**
 * @brief Calibrates the gyroscope by calculating and setting hardware offsets.
 * @note The sensor MUST be stationary on a flat surface during calibration.
 * @param mpu Pointer to the mpu6500_t struct.
 * @param num_samples The number of readings to average for calibration. (e.g., 200)
 */
void mpu6500_calibrate_gyro(mpu6500_t *mpu, uint16_t num_samples);

/**
 * @brief Calibrates the accelerometer by calculating and setting hardware offsets.
 * @note The sensor MUST be stationary on a flat surface during calibration.
 * @param mpu Pointer to the mpu6500_t struct.
 * @param num_samples The number of readings to average for calibration. (e.g., 200)
 */
void mpu6500_calibrate_accel(mpu6500_t *mpu, uint16_t num_samples);

/**
 * @brief Initializes the odometry structure.
 *
 * @param odom Pointer to the mpu6500_odometry_t struct.
 */
void mpu6500_odometry_init(mpu6500_t *mpu);

/**
 * @brief Updates the odometry based on accelerometer and gyroscope data.
 *
 * @param odom Pointer to the mpu6500_odometry_t struct.
 * @param mpu Pointer to the mpu6500_t struct.
 * @param current_time_us Current time in microseconds.
 */
void mpu6500_odometry_speed_update(mpu6500_t *mpu, uint64_t current_time_us);


void mpu6500_odometry_position_update(mpu6500_t *mpu, uint64_t current_time_us);

void imu_odo_loop(mpu6500_t *mpu);


void reset_odometer_position(mpu6500_t *mpu);

#endif // MPU6500_H
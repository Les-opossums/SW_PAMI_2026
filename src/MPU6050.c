#include "PAMI_2026.h"
#include <math.h> // NOTE: Added for sqrtf if not already included globally

// NOTE: Most register definitions are the same between MPU-6050 and MPU-6500.
// These are all correct.
#define ACCEL_XOFFS_H 0x06
#define ACCEL_XOFFS_L 0x07
#define ACCEL_YOFFS_H 0x08
#define ACCEL_YOFFS_L 0x09
#define ACCEL_ZOFFS_H 0x0A
#define ACCEL_ZOFFS_L 0x0B
#define GYRO_XOFFS_H 0x13
#define GYRO_XOFFS_L 0x14
#define GYRO_YOFFS_H 0x15
#define GYRO_YOFFS_L 0x16
#define GYRO_ZOFFS_H 0x17
#define GYRO_ZOFFS_L 0x18
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B   // Gyroscope Configuration
#define ACCEL_CONFIG 0x1C  // Accelerometer Configuration
#define ACCEL_CONFIG_2 0x1D // NOTE: MPU-6500 has a second accel config register
#define INT_PIN_CFG 0x37   // INT Pin. Bypass Enable Configuration
#define INT_ENABLE 0x38    // INT Enable
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define USER_CTRL 0x6A   // User Control
#define PWR_MGMT_1 0x6B  // Power Management 1
#define PWR_MGMT_2 0x6C  // Power Management 2
#define WHO_AM_I 0x75    // Who Am I

#define GRAVITY_CONSTANT 9.80665f

// Helper function to write a single byte to a register
void i2c_write_reg_byte(struct i2c_information *i2c, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(i2c->instance, i2c->address, data, 2, false);
}

int i2c_read_reg(struct i2c_information *i2c, const uint8_t reg, uint8_t *buf, const size_t len)
{
    i2c_write_blocking(i2c->instance, i2c->address, &reg, 1, true);
    return i2c_read_blocking(i2c->instance, i2c->address, buf, len, false);
}

void read_raw_gyro(struct mpu6050 *self)
{
    uint8_t data[6];
    i2c_read_reg(&self->i2c, GYRO_XOUT_H, data, 6);

    self->rg.x = (int16_t)(data[0] << 8 | data[1]);
    self->rg.y = (int16_t)(data[2] << 8 | data[3]);
    self->rg.z = (int16_t)(data[4] << 8 | data[5]);
}

void read_raw_accel(struct mpu6050 *self)
{
    uint8_t data[6];
    i2c_read_reg(&self->i2c, ACCEL_XOUT_H, data, 6);

    self->ra.x = (int16_t)(data[0] << 8 | data[1]);
    self->ra.y = (int16_t)(data[2] << 8 | data[3]);
    self->ra.z = (int16_t)(data[4] << 8 | data[5]);
}

inline static void i2c_write_u16_inline(struct i2c_information *i2c, uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {reg, (value >> 8), (value & 0xFF)};
    i2c_write_blocking(i2c->instance, i2c->address, data, 3, false);
}

inline static void i2c_write_bit_in_reg_inline(struct i2c_information *i2c, uint8_t reg, uint8_t pos, uint8_t state)
{
    uint8_t reg_value;
    i2c_read_reg(i2c, reg, &reg_value, 1);

    if (state) {
        reg_value |= (1 << pos);
    } else {
        reg_value &= ~(1 << pos);
    }
    i2c_write_reg_byte(i2c, reg, reg_value);
}

struct mpu6050 mpu6050_init(i2c_inst_t *i2c_instance, const uint8_t address)
{
    struct mpu6050 mpu6050;
    mpu6050.i2c.instance = i2c_instance;
    mpu6050.i2c.address = address;
    // ... (rest of the init is fine)
    mpu6050.calibration_data.dg.x = 0;
    mpu6050.calibration_data.dg.y = 0;
    mpu6050.calibration_data.dg.z = 0;
    mpu6050.config.use_calibrate = 0;
    mpu6050.calibration_data.tg.x = 0;
    mpu6050.calibration_data.tg.y = 0;
    mpu6050.calibration_data.tg.z = 0;
    mpu6050.config.actual_threshold = 0;
    mpu6050.config.meas_temp = 0;
    mpu6050.config.meas_acce = 0;
    mpu6050.config.meas_gyro = 0;
    return mpu6050;
}

uint8_t mpu6050_who_am_i(struct mpu6050 *self)
{
    uint8_t who_am_i;
    i2c_read_reg(&self->i2c, WHO_AM_I, &who_am_i, 1);
    return who_am_i;
}

uint8_t mpu6050_begin(struct mpu6050 *self)
{
    // MODIFIED: Add a reset sequence for MPU-6500 stability.
    // Reset the device.
    i2c_write_reg_byte(&self->i2c, PWR_MGMT_1, 0x80);
    sleep_ms(100);
    // Wake device and set clock source to Gyro Z reference.
    i2c_write_reg_byte(&self->i2c, PWR_MGMT_1, 0x03);
    sleep_ms(10);

    uint8_t who_am_i = mpu6050_who_am_i(self);

    // MODIFIED: Print a more generic name and check for the MPU-6500 ID.
    printf("MPU IMU Address: 0x%02X, WHO_AM_I: 0x%02X\n", self->i2c.address, who_am_i);
    if (who_am_i != 0x70) { // MODIFIED: Changed from 0x68 to 0x70 for MPU-6500.
        return 0;
    }

    // Default configuration
    mpu6050_set_range(self, MPU6050_RANGE_4G);
    mpu6050_set_scale(self, MPU6050_SCALE_500DPS);

    return 1;
}

// NOTE: This function reads registers that are NOT present on the MPU-6500.
// We are commenting out the incompatible parts.
uint8_t mpu6050_event(struct mpu6050 *self)
{
    self->rg.x = 0; self->rg.y = 0; self->rg.z = 0;
    self->ra.x = 0; self->ra.y = 0; self->ra.z = 0;
    self->ng.x = 0.0f; self->ng.y = 0.0f; self->ng.z = 0.0f;
    self->na.x = 0.0f; self->na.y = 0.0f; self->na.z = 0.0f;
    self->raw_temperature = 0;
    self->temperature = 0.0f;
    self->temperaturef = 0.0f;

    if (self->config.meas_gyro) { read_raw_gyro(self); }
    if (self->config.meas_acce) { read_raw_accel(self); }

    if (self->config.meas_temp) {
        uint8_t data[2];
        i2c_read_reg(&self->i2c, TEMP_OUT_H, data, 2);
        self->raw_temperature = (int16_t)(data[0] << 8 | data[1]);
    }

    uint8_t int_status;
    i2c_read_reg(&self->i2c, INT_STATUS, &int_status, 1);

    // The INT_STATUS register is different.
    // MODIFIED: Simplified to only check the Data Ready flag, which is bit 0.
    self->activity.isDataReady = (int_status & 0x01);
    self->activity.isOverflow = 0;
    self->activity.isFreefall = 0;
    self->activity.isInactivity = 0;
    self->activity.isActivity = 0;

    // MODIFIED: The MOT_DETECT_STATUS register (0x61) does not exist on the MPU-6500.
    // This part has been removed.
    self->activity.isNegActivityOnX = 0;
    self->activity.isPosActivityOnX = 0;
    self->activity.isNegActivityOnY = 0;
    self->activity.isPosActivityOnY = 0;
    self->activity.isNegActivityOnZ = 0;
    self->activity.isPosActivityOnZ = 0;
    
    return 0; // Return value was unused
}

// NOTE: Sensitivity factors are the same as MPU-6050, so this function is correct.
void mpu6050_set_scale(struct mpu6050 *self, enum MPU6050_SCALE scale)
{
    switch (scale) {
        case MPU6050_SCALE_250DPS:  self->config.dps_per_digit = 1.0f / 131.0f; break;
        case MPU6050_SCALE_500DPS:  self->config.dps_per_digit = 1.0f / 65.5f; break;
        case MPU6050_SCALE_1000DPS: self->config.dps_per_digit = 1.0f / 32.8f; break;
        case MPU6050_SCALE_2000DPS: self->config.dps_per_digit = 1.0f / 16.4f; break;
    }
    uint8_t gyro_config;
    i2c_read_reg(&self->i2c, GYRO_CONFIG, &gyro_config, 1);
    gyro_config &= 0xE7;
    gyro_config |= (scale << 3);
    i2c_write_reg_byte(&self->i2c, GYRO_CONFIG, gyro_config);
}

// NOTE: Sensitivity factors are the same as MPU-6050, so this function is correct.
void mpu6050_set_range(struct mpu6050 *self, enum MPU6050_RANGE range)
{
    switch (range) {
        case MPU6050_RANGE_2G:  self->config.range_per_digit = 1.0f / 16384.0f; break;
        case MPU6050_RANGE_4G:  self->config.range_per_digit = 1.0f / 8192.0f; break;
        case MPU6050_RANGE_8G:  self->config.range_per_digit = 1.0f / 4096.0f; break;
        case MPU6050_RANGE_16G: self->config.range_per_digit = 1.0f / 2048.0f; break;
    }
    uint8_t accel_config;
    i2c_read_reg(&self->i2c, ACCEL_CONFIG, &accel_config, 1);
    accel_config &= 0xE7;
    accel_config |= (range << 3);
    i2c_write_reg_byte(&self->i2c, ACCEL_CONFIG, accel_config);
}

// NOTE: `mpu6050_set_sleep_enabled` from original code was buggy.
// This is the correct way to set/clear a bit in a register.
void mpu6050_set_sleep_enabled(struct mpu6050 *self, uint8_t state)
{
    i2c_write_bit_in_reg_inline(&self->i2c, PWR_MGMT_1, 6, state);
}


// ... (calibration and data processing functions are largely okay) ...
// The following data processing functions are fine as they operate on stored data.
void mpu6050_calibrate_gyro(struct mpu6050 *self, uint8_t samples)
{
    self->config.use_calibrate = 1;
    float sumX = 0, sumY = 0, sumZ = 0;
    float sigmaX = 0, sigmaY = 0, sigmaZ = 0;

    for (uint8_t i = 0; i < samples; i++) {
        read_raw_gyro(self);
        sumX += self->rg.x; sumY += self->rg.y; sumZ += self->rg.z;
        sigmaX += self->rg.x * self->rg.x;
        sigmaY += self->rg.y * self->rg.y;
        sigmaZ += self->rg.z * self->rg.z;
        sleep_ms(5);
    }

    self->calibration_data.dg.x = sumX / samples;
    self->calibration_data.dg.y = sumY / samples;
    self->calibration_data.dg.z = sumZ / samples;

    self->calibration_data.th.x = sqrtf((sigmaX / samples) - (self->calibration_data.dg.x * self->calibration_data.dg.x));
    self->calibration_data.th.y = sqrtf((sigmaY / samples) - (self->calibration_data.dg.y * self->calibration_data.dg.y));
    self->calibration_data.th.z = sqrtf((sigmaZ / samples) - (self->calibration_data.dg.z * self->calibration_data.dg.z));

    if (self->config.actual_threshold > 0) {
        mpu6050_set_threshold(self, 1);
    }
}

void mpu6050_set_threshold(struct mpu6050 *self, uint8_t multiple)
{
    if (multiple > 0) {
        if (!self->config.use_calibrate) {
            mpu6050_calibrate_gyro(self, 50);
        }
        self->calibration_data.tg.x = self->calibration_data.th.x * multiple;
        self->calibration_data.tg.y = self->calibration_data.th.y * multiple;
        self->calibration_data.tg.z = self->calibration_data.th.z * multiple;
    } else {
        self->calibration_data.tg.x = 0;
        self->calibration_data.tg.y = 0;
        self->calibration_data.tg.z = 0;
    }
    self->config.actual_threshold = multiple;
}

struct mpu6050_vectorf *mpu6050_get_accelerometer(struct mpu6050 *self)
{
    self->na.x = self->ra.x * self->config.range_per_digit * GRAVITY_CONSTANT;
    self->na.y = self->ra.y * self->config.range_per_digit * GRAVITY_CONSTANT;
    self->na.z = self->ra.z * self->config.range_per_digit * GRAVITY_CONSTANT;
    return &self->na;
}

struct mpu6050_vectorf *mpu6050_get_gyroscope(struct mpu6050 *self)
{
    if (self->config.use_calibrate) {
        self->ng.x = (self->rg.x - self->calibration_data.dg.x) * self->config.dps_per_digit;
        self->ng.y = (self->rg.y - self->calibration_data.dg.y) * self->config.dps_per_digit;
        self->ng.z = (self->rg.z - self->calibration_data.dg.z) * self->config.dps_per_digit;
    } else {
        self->ng.x = self->rg.x * self->config.dps_per_digit;
        self->ng.y = self->rg.y * self->config.dps_per_digit;
        self->ng.z = self->rg.z * self->config.dps_per_digit;
    }

    if (self->config.actual_threshold) {
        if (fabs(self->ng.x) < self->calibration_data.tg.x) self->ng.x = 0.0f;
        if (fabs(self->ng.y) < self->calibration_data.tg.y) self->ng.y = 0.0f;
        if (fabs(self->ng.z) < self->calibration_data.tg.z) self->ng.z = 0.0f;
    }
    return &self->ng;
}

// MODIFIED: Corrected the temperature formula for the MPU-6500.
float mpu6050_get_temperature_c(struct mpu6050 *self)
{
    // Formula for MPU-6500 is: Temp_C = (Temp_Raw / 333.87) + 21.0
    self->temperature = ((float)self->raw_temperature / 333.87f) + 21.0f;
    return self->temperature;
}

float mpu6050_get_temperature_f(struct mpu6050 *self)
{
    if (self->temperature == 0.0f) {
        mpu6050_get_temperature_c(self);
    }
    self->temperaturef = (self->temperature * 1.8f) + 32.0f;
    return self->temperaturef;
}

// NOTE: Offset registers are present on MPU-6500, these functions are fine.
void mpu6050_set_gyro_offset_x(struct mpu6050 *self, uint16_t offset) { i2c_write_u16_inline(&self->i2c, GYRO_XOFFS_H, offset); }
void mpu6050_set_gyro_offset_y(struct mpu6050 *self, uint16_t offset) { i2c_write_u16_inline(&self->i2c, GYRO_YOFFS_H, offset); }
void mpu6050_set_gyro_offset_z(struct mpu6050 *self, uint16_t offset) { i2c_write_u16_inline(&self->i2c, GYRO_ZOFFS_H, offset); }
void mpu6050_set_accel_offset_x(struct mpu6050 *self, uint16_t offset) { i2c_write_u16_inline(&self->i2c, ACCEL_XOFFS_H, offset); }
void mpu6050_set_accel_offset_y(struct mpu6050 *self, uint16_t offset) { i2c_write_u16_inline(&self->i2c, ACCEL_YOFFS_H, offset); }
void mpu6050_set_accel_offset_z(struct mpu6050 *self, uint16_t offset) { i2c_write_u16_inline(&self->i2c, ACCEL_ZOFFS_H, offset); }


// The following functions are INCOMPATIBLE with the MPU-6500 as they use registers
// that do not exist on this chip. Their bodies have been commented out to prevent errors.
// Implementing the MPU-6500's "Wake-on-Motion" would require new functions and logic.

void mpu6050_set_int_motion(struct mpu6050 *self, uint8_t state) {
    // NOTE: Incompatible with MPU-6500. This uses INT_ENABLE bit 6.
    // On MPU-6500, this is the "Wake-on-Motion Interrupt Enable" bit.
    // The underlying hardware logic is completely different.
}
void mpu6050_set_int_zero_motion(struct mpu6050 *self, uint8_t state) {
    // NOTE: Incompatible with MPU-6500. This register (bit 5) does not exist.
}
void mpu6050_set_int_free_fall(struct mpu6050 *self, uint8_t state) {
    // NOTE: Incompatible with MPU-6500. This register (bit 7) does not exist.
}
void mpu6050_set_motion_detection_threshold(struct mpu6050 *self, uint8_t threshold) {
    // NOTE: Incompatible with MPU-6500. Register 0x1F (MOT_THRESHOLD) does not exist.
}
void mpu6050_set_motion_detection_duration(struct mpu6050 *self, uint8_t duration) {
    // NOTE: Incompatible with MPU-6500. Register 0x20 (MOT_DURATION) does not exist.
}
void mpu6050_set_zero_motion_detection_threshold(struct mpu6050 *self, uint8_t threshold) {
    // NOTE: Incompatible with MPU-6500. Register 0x21 (ZMOT_THRESHOLD) does not exist.
}
void mpu6050_set_zero_motion_detection_duration(struct mpu6050 *self, uint8_t duration) {
    // NOTE: Incompatible with MPU-6500. Register 0x22 (ZMOT_DURATION) does not exist.
}
void mpu6050_set_free_fall_detection_threshold(struct mpu6050 *self, uint8_t threshold) {
    // NOTE: Incompatible with MPU-6500. Register 0x1D (FF_THRESHOLD) is ACCEL_CONFIG_2 on the 6500.
}
void mpu6050_set_free_fall_detection_duration(struct mpu6050 *self, uint8_t duration) {
    // NOTE: Incompatible with MPU-6500. Register 0x1E (FF_DURATION) does not exist.
}

// These functions remain compatible
void mpu6050_set_dlpf_mode(struct mpu6050 *self, enum MPU6050_DLPF dlpf)
{
    uint8_t value;
    i2c_read_reg(&self->i2c, CONFIG, &value, 1);
    value &= 0xF8;
    value |= dlpf;
    i2c_write_reg_byte(&self->i2c, CONFIG, value);
}

// These are simple setters/getters that are fine
void mpu6050_set_temperature_measuring(struct mpu6050 *self, uint8_t state) { self->config.meas_temp = state; }
void mpu6050_set_accelerometer_measuring(struct mpu6050 *self, uint8_t state) { self->config.meas_acce = state; }
void mpu6050_set_gyroscope_measuring(struct mpu6050 *self, uint8_t state) { self->config.meas_gyro = state; }
struct mpu6050_activity *mpu6050_read_activities(struct mpu6050 *self) { return &self->activity; }
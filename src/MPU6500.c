#include "PAMI_2026.h"

mpu6500_t mpu6500;

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

    gpio_init(IMU_I2C_SDA);
    gpio_init(IMU_I2C_SCL);
    i2c_init(i2c1, 400000); // 400kHz

    gpio_set_function(IMU_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL, GPIO_FUNC_I2C);
    // Don't forget the pull ups! | Or use external ones
    gpio_pull_up(IMU_I2C_SDA);
    gpio_pull_up(IMU_I2C_SCL);

    // Attempt to begin communication with the MPU-6500
    if (!mpu6500_begin(mpu)) {
        printf("Failed to initialize MPU-6500. Halting.\n");
        while (1); // Loop forever if initialization fails
    }

    mpu6500_calibrate_gyro(mpu, 1000); // Calibrate using 1000 samples
    mpu6500_calibrate_accel(mpu, 1000); // Calibrate using 1000 samples
    printf("Calibration finished. Starting main loop.\n\n");
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

void mpu6500_odometry_init(mpu6500_t *mpu)
{
    mpu->odom.x = 0.0f;
    mpu->odom.y = 0.0f;
    mpu->odom.theta = 0.0f;

    mpu->odom.vx = 0.0f;
    mpu->odom.vy = 0.0f;
    mpu->odom.wz = 0.0f;

    mpu->odom.cumul_vx = 0.0f;
    mpu->odom.cumul_vy = 0.0f;
    mpu->odom.cumul_wz = 0.0f;

    mpu->odom.g_est_x = 0.0f;
    mpu->odom.g_est_y = 0.0f;
    mpu->odom.g_est_z = 0.0f;

    mpu->odom.pitch = 0.0f;
    mpu->odom.roll = 0.0f;

    mpu->odom.accel_bias_x = 0.0f;
    mpu->odom.accel_bias_y = 0.0f;
    mpu->odom.gyro_bias_z = 0.0f;

    mpu->odom.speed_last_time_us = 0;
    mpu->odom.position_last_time_us = 0;

    mpu->odom.speed_update_counter = 0;
}

void mpu6500_odometry_speed_update(mpu6500_t *mpu, uint64_t current_time_us)
{
    mpu6500_odometry_t *odom = &mpu->odom;

    // 1. Calculate time delta (dt) in seconds.
    float dt = (current_time_us - odom->speed_last_time_us) / 1e6f;
    odom->speed_last_time_us = current_time_us;

    // Safety check for valid time step
    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }

    // 2. Get calibrated sensor data.
    mpu6500_float_data_t accel_data;
    mpu6500_float_data_t gyro_data;
    mpu6500_get_accel_g(mpu, &accel_data);
    mpu6500_get_gyro_dps(mpu, &gyro_data);

    // 3. Attitude Estimation via Complementary Filter
    //  a) Integrate gyro data for a fast, short-term estimate of orientation.
    //     NOTE: The sign of gy_rad might need to be flipped depending on your IMU's axis orientation.
    
    float gyro_rad_x = gyro_data.x * (M_PI / 180.0f); // Convert dps to rad/s
    float gyro_rad_y = gyro_data.y * (M_PI / 180.0f);
    float gyro_rad_z = gyro_data.z * (M_PI / 180.0f);

    odom->pitch += gyro_rad_x * dt;
    odom->roll  -= gyro_rad_y * dt;

    float ax = accel_data.x * G_ACCEL;
    float ay = accel_data.y * G_ACCEL;
    float az = accel_data.z * G_ACCEL;

    //  b) Calculate long-term orientation from the accelerometer (which way is "down").
    float pitch_accel = atan2f(ax, sqrtf(ay * ay + az * az));
    float roll_accel  = atan2f(-ay, -az);

    //  c) Fuse the two estimates.
    odom->pitch = COMPLEMENTARY_FILTER_ALPHA * odom->pitch + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * pitch_accel;
    odom->roll  = COMPLEMENTARY_FILTER_ALPHA * odom->roll  + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * roll_accel;

    // 4. Gravity Compensation
    //  Use the estimated attitude to calculate and remove the gravity component.
    float gravity_comp_x =  G_ACCEL * sinf(odom->pitch);
    float gravity_comp_y = -G_ACCEL * sinf(odom->roll) * cosf(odom->pitch);

    float linear_ax = ax - gravity_comp_x;
    float linear_ay = ay - gravity_comp_y;

    // 5. Integrate corrected acceleration to get velocity.
    odom->vx += linear_ax * dt;
    odom->vy += linear_ay * dt;
    odom->wz = gyro_rad_z; // Yaw rate is directly from the gyro

    // printf("speed vx=%.2f m/s, vy=%.2f m/s, wz=%.2f rad/s\n", odom->vx, odom->vy, odom->wz);

     // Optional: small acceleration damping
    float acc_mag = sqrtf(linear_ax*linear_ax + linear_ay*linear_ay);
    if (acc_mag < ACC_THRESHOLD) {
        odom->vx *= VEL_DECAY;
        odom->vy *= VEL_DECAY;
    }

    // 6. Accumulate velocities for the lower-frequency position update.
    odom->cumul_vx += odom->vx;
    odom->cumul_vy += odom->vy;
    odom->cumul_wz += odom->wz;
}

/**
 * @brief Updates position by integrating the averaged velocities.
 *
 * This function should be called at a lower frequency than the speed update.
 * It takes the accumulated velocity, averages it, and computes the change in position.
 */
void mpu6500_odometry_position_update(mpu6500_t *mpu, uint64_t current_time_us)
{
    // Calculate the total time elapsed since the last position update.
    // float dt_total = (current_time_us - mpu->odom.position_last_time_us) / 1e6f;
    // mpu->odom.position_last_time_us = current_time_us;

    // if (dt_total <= 0.0f) {
    //     return;
    // }
    float dt = 0.001f * ODO_POS_EVERY_SPEED; // Assuming speed update is at 1ms intervals

    // Average the velocities collected since the last position update.
    float vx_avg = mpu->odom.cumul_vx / ODO_POS_EVERY_SPEED;
    float vy_avg = mpu->odom.cumul_vy / ODO_POS_EVERY_SPEED;
    float wz_avg = mpu->odom.cumul_wz / ODO_POS_EVERY_SPEED;

    // Reset cumulative sums for the next cycle.
    mpu->odom.cumul_vx = 0.0f;
    mpu->odom.cumul_vy = 0.0f;
    mpu->odom.cumul_wz = 0.0f;

    // Calculate displacement in the robot's local frame.
    float dx_local = vx_avg * dt;
    float dy_local = vy_avg * dt;
    float dtheta   = wz_avg * dt;

    // Rotate the local displacement into the world frame and update the position.
    float cos_theta = cosf(mpu->odom.theta);
    float sin_theta = sinf(mpu->odom.theta);

    mpu->odom.x += cos_theta * dx_local - sin_theta * dy_local;
    mpu->odom.y += sin_theta * dx_local + cos_theta * dy_local;
    mpu->odom.theta += dtheta;

    // Normalize theta to the range [-PI, PI] to prevent wrap-around errors.
    mpu->odom.theta = atan2f(sinf(mpu->odom.theta), cosf(mpu->odom.theta));
}

int state_imu_odo = 0;
uint64_t imu_timer = 0;

void imu_odo_loop(mpu6500_t *mpu)
{
    switch (state_imu_odo) {
        case 0:
            if (Timer_ms1 - imu_timer >= 1) { // 1000 Hz
                imu_timer = Timer_ms1;
                mpu6500_odometry_speed_update(mpu, time_us_64());
                // printf("Speed: vx=%.2f m/s, vy=%.2f m/s, wz=%.2f rad/s\n", mpu->odom.vx, mpu->odom.vy, mpu->odom.wz);
                mpu->odom.speed_update_counter++;
                if (mpu->odom.speed_update_counter >= ODO_POS_EVERY_SPEED) {
                    mpu->odom.speed_update_counter = 0;
                    state_imu_odo++; // Transition to position update
                }
            }
            break;
        case 1:
            mpu6500_odometry_position_update(mpu, time_us_64());
            printf("Odometry: X=%.2f m, Y=%.2f m, Theta=%.2f rad\n", mpu->odom.x, mpu->odom.y, mpu->odom.theta);
            state_imu_odo++;
            break;
        default:
            state_imu_odo = 0;
            break;
    }
}


void reset_odometer_position(mpu6500_t *mpu)
{
    mpu->odom.x = 0.0f;
    mpu->odom.y = 0.0f;
    mpu->odom.theta = 0.0f;
}

uint8_t SET0_imu_cmd(void) {
    reset_odometer_position(&mpu6500);
    return 0;
}
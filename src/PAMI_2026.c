#include "PAMI_2026.h"

int main()
{
    stdio_init_all();
    
    int sequencer = 0;

    Init_All();

    
    // float x = 0, y = 0, theta = 0;
    printf("Starting MPU-6500 configuration...\n");
    // Init of the MPU6500
    mpu6500_init(&mpu6500, i2c1, MPU6500_I2C_ADDR); // Most common I2C address for MPU-6500 is 0x68
    // Data structures to hold the sensor readings
    mpu6500_float_data_t accel_data;
    mpu6500_float_data_t gyro_data;

    
    mpu6500_odometry_init(&mpu6500);
    float temp_c;

    while (true) {
        Timer_Update(); // Met à jour les timers
        int c;

        // Read all raw sensor data at once
        mpu6500_read_raw(&mpu6500);

        // // Convert raw data to meaningful units
        mpu6500_get_accel_g(&mpu6500, &accel_data);
        mpu6500_get_gyro_dps(&mpu6500, &gyro_data);

        // // Print the results
        // printf("Accel (g): X=%.2f, Y=%.2f, Z=%.2f  |  ", accel_data.x, accel_data.y, accel_data.z);
        // printf("Gyro (dps): X=%.2f, Y=%.2f, Z=%.2f  |  ", gyro_data.x, gyro_data.y, gyro_data.z);

        // Met à jour les moteurs pas à pas
        //     Move_Loop();

        switch (sequencer) {
            case 0:
                c = getchar_timeout_us(0);
				if (c >= 0) {
					Interp(c);
				}
                sequencer++;
                break;
            case 1:
                // Asserv_Loop();
                sequencer++;
                break;
            case 2:
                imu_odo_loop(&mpu6500);
                sequencer++;
                break;
            default:
                sequencer = 0;
                break;
        }
    // }
    // cyw43_arch_deinit();
    }
    return 0;
}

void Init_All(void)
{
    init_motors();
    Init_Asserv();
}


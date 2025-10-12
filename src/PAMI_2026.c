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

        // Met à jour les moteurs pas à pas
        // Move_Loop();

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


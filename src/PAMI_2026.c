#include "PAMI_2026.h"

int main()
{
    stdio_init_all();
    
    int sequencer = 0;

    Init_All();

    printf("PAMI-2026 ready.\n");

    // char ssid[] = "Opossum";           // your network SSID (name)
    // char password[] = "r28w3fr7j3zu8r4";       // your network password

    // // Initialise la puce CYW43 (WiFi + Bluetooth)
    // if (cyw43_arch_init_with_country(CYW43_COUNTRY_FRANCE)) {
    //     printf("failed to initialise\n");
    //     return 1;
    // }
    
    // cyw43_arch_enable_sta_mode();

    // if (cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
    //     printf("wifi connection failed\n");
    //     return 1;
    // }
    // printf("IP address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));

    // tcp_server_t *server = tcp_server_open(); 
    // if (!server) {
    //     printf("Failed to start TCP server\n");
    //     while(1); // Halt
    // }

    // uint32_t tcp_timer = to_ms_since_boot(get_absolute_time());


    // float x = 0, y = 0, theta = 0;
    gpio_init(IMU_I2C_SDA);
    gpio_init(IMU_I2C_SCL);
    i2c_init(i2c1, 400000); // 400kHz


    gpio_set_function(IMU_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL, GPIO_FUNC_I2C);
    // Don't forget the pull ups! | Or use external ones
    gpio_pull_up(IMU_I2C_SDA);
    gpio_pull_up(IMU_I2C_SCL);

    sleep_ms(2000); // give time for USB to connect
    printf("Starting I2C scan...\n");
    // for (uint8_t addr = 1; addr < 127; addr++) {
    //     // Dummy write, no data, just check for ACK
    //     uint8_t rxdata;
    //     int result = i2c_read_blocking(i2c1   , addr, &rxdata, 1, false);
    //     if (result >= 0) {
    //         printf("✅ Found device at 0x%02X\n", addr);
    //     }
    // }
    // printf("Scan complete.\n");

    // while(1);
    mpu6050_t mpu6050 = mpu6050_init(i2c1, 0x68);


    if (mpu6050_begin(&mpu6050))
    {
        // Set scale of gyroscope
        mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);
        // Set range of accelerometer
        mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);

        // Enable temperature, gyroscope and accelerometer readings
        mpu6050_set_temperature_measuring(&mpu6050, true);
        mpu6050_set_gyroscope_measuring(&mpu6050, true);
        mpu6050_set_accelerometer_measuring(&mpu6050, true);

        // Enable free fall, motion and zero motion interrupt flags
        mpu6050_set_int_free_fall(&mpu6050, false);
        mpu6050_set_int_motion(&mpu6050, false);
        mpu6050_set_int_zero_motion(&mpu6050, false);

        // Set motion detection threshold and duration
        mpu6050_set_motion_detection_threshold(&mpu6050, 2);
        mpu6050_set_motion_detection_duration(&mpu6050, 5);

        // Set zero motion detection threshold and duration
        mpu6050_set_zero_motion_detection_threshold(&mpu6050, 4);
        mpu6050_set_zero_motion_detection_duration(&mpu6050, 2);
    }
    else
    {
        while (1)
        {
            // Endless loop
            printf("Error! MPU6050 could not be initialized. Make sure you've entered the correct address. And double check your connections.\n");
            sleep_ms(500);
        }
    }

    while (true) {
        Timer_Update(); // Met à jour les timers
        int c;

        mpu6050_event(&mpu6050);

        // Pointers to float vectors with all the results
        mpu6050_vectorf_t *accel = mpu6050_get_accelerometer(&mpu6050);
        mpu6050_vectorf_t *gyro = mpu6050_get_gyroscope(&mpu6050);

        // Activity struct holding all interrupt flags
        mpu6050_activity_t *activities = mpu6050_read_activities(&mpu6050);

        // Rough temperatures as float -- Keep in mind, this is not a temperature sensor!!!
        float tempC = mpu6050_get_temperature_c(&mpu6050);
        float tempF = mpu6050_get_temperature_f(&mpu6050);

        // Print all the measurements
        printf("Accelerometer: %f, %f, %f \n", accel->x, accel->y, accel->z);

        // Print all motion interrupt flags
        // printf("Overflow: %d - Freefall: %d - Inactivity: %d, Activity: %d, DataReady: %d\n",
        //        activities->isOverflow,
        //        activities->isFreefall,
        //        activities->isInactivity,
        //        activities->isActivity,
        //        activities->isDataReady);

        // // Print all motion detect interrupt flags
        // printf("PosX: %d - NegX: %d -- PosY: %d - NegY: %d -- PosZ: %d - NegZ: %d\n",
        //        activities->isPosActivityOnX,
        //        activities->isNegActivityOnX,
        //        activities->isPosActivityOnY,
        //        activities->isNegActivityOnY,
        //        activities->isPosActivityOnZ,
        //        activities->isNegActivityOnZ);
        // cyw43_arch_poll();
        
        // if ((Timer_ms1 - tcp_timer) > 1000) {

        //     // printf("client_pcb=%p, is_handshake_complete=%d\n", server->client_pcb, server->complete);

        //     char msg[128];
        //     snprintf(msg, sizeof(msg), "{\"x\":%.3f,\"y\":%.3f,\"theta\":%.3f}\n\0", x, y, theta);
        //     tcp_server_send_data(server, (uint8_t*)msg, strlen(msg));
            
        //     // NEW: Call the Foxglove sender function
        //     // send_pose_to_foxglove(server, x, y, theta);

        //     // simulation d’évolution
        //     x += 0.05f;
        //     theta += 0.01f;

        //     tcp_timer = to_ms_since_boot(get_absolute_time());
        // }

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


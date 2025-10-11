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

    mpu6500_t mpu6500;
    mpu6500_init(&mpu6500, i2c1, MPU6500_I2C_ADDR); // Most common I2C address for MPU-6500 is 0x68
    // Attempt to begin communication with the MPU-6500
    if (!mpu6500_begin(&mpu6500)) {
        printf("Failed to initialize MPU-6500. Halting.\n");
        while (1); // Loop forever if initialization fails
    }

    mpu6500_calibrate_gyro(&mpu6500, 1000); // Calibrate using 1000 samples
    mpu6500_calibrate_accel(&mpu6500, 1000); // Calibrate using 1000 samples
    printf("Calibration finished. Starting main loop.\n\n");

    // Data structures to hold the sensor readings
    mpu6500_float_data_t accel_data;
    mpu6500_float_data_t gyro_data;
    float temp_c;


    while (true) {
        Timer_Update(); // Met à jour les timers
        int c;

        // Read all raw sensor data at once
        mpu6500_read_raw(&mpu6500);

        // // Convert raw data to meaningful units
        mpu6500_get_accel_g(&mpu6500, &accel_data);
        mpu6500_get_gyro_dps(&mpu6500, &gyro_data);
        temp_c = mpu6500_get_temp_c(&mpu6500);

        // // Print the results
        printf("Accel (g): X=%.2f, Y=%.2f, Z=%.2f  |  ", accel_data.x, accel_data.y, accel_data.z);
        printf("Gyro (dps): X=%.2f, Y=%.2f, Z=%.2f  |  ", gyro_data.x, gyro_data.y, gyro_data.z);
        printf("Temp: %.2f C\n", temp_c);
        
        // sleep_ms(100); // Delay for readability

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


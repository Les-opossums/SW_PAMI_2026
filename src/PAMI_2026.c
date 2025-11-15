#include "PAMI_2026.h"

// Pin configuration (Matches the table above)
#define LCD_CS_PIN 17
#define LCD_DC_PIN 16
#define LCD_RST_PIN 15 // Use -1 if you skip the reset pin

// Global display structure
gc9a01a_t tft;

int main() {
    stdio_init_all();
    printf("GC9A01A Display Driver Test\n");

    // 1. Initialize the C structure with pin numbers
    gc9a01a_init_struct(&tft, LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN);

    // 2. Initialize the display (includes SPI setup and command sequence)
    // We pass 40MHz as the desired frequency
    gc9a01a_begin(&tft, 40000000);

    // 3. Set up the graphics properties
    gc9a01a_set_rotation(&tft, 1); // Landscape mode

    // 4. Draw a solid color (Example of writing to the display)
    // Set the full screen as the address window
    gc9a01a_set_addr_window(&tft, 0, 0, tft._width, tft._height);

    // Write a large block of Red color data (GC9A01A_RED is 0xF800)
    uint16_t color = GC9A01A_RED;
    // Prepare the pixel data (two bytes for red, repeated for a small chunk)
    uint8_t pixel_buffer[64];
    for (int i = 0; i < 64; i += 2) {
        pixel_buffer[i] = (uint8_t)(color >> 8);   // MSB
        pixel_buffer[i+1] = (uint8_t)(color & 0xFF); // LSB
    }

    // Write all pixels (240*240 total)
    size_t total_pixels = tft._width * tft._height;
    size_t i = 0;

    // CS must be low for the entire RAMWR data stream
    gpio_put(tft.cs_pin, 0); 
    gpio_put(tft.dc_pin, 1); // Data mode

    while (i < total_pixels * 2) { // total bytes to write
        // Determine how many bytes to write (up to 64)
        size_t write_len = (total_pixels * 2 - i) > 64 ? 64 : (total_pixels * 2 - i);
        
        // Write the chunk of data (pixels)
        spi_write_blocking(tft.spi_instance, pixel_buffer, write_len);
        i += write_len;
    }

    // Deselect the chip
    gpio_put(tft.cs_pin, 1);

    printf("Display filled with Red. Done.\n");

    while (true) {
        // Your main loop logic here
        sleep_ms(1000);
    }
}



// int main()
// {
//     stdio_init_all();
    
//     int sequencer = 0;

//     Init_All();

//     printf("PAMI-2026 ready.\n");

//     char ssid[] = "Opossum";           // your network SSID (name)
//     char password[] = "r28w3fr7j3zu8r4";       // your network password

//     // Initialise la puce CYW43 (WiFi + Bluetooth)
//     if (cyw43_arch_init_with_country(CYW43_COUNTRY_FRANCE)) {
//         printf("failed to initialise\n");
//         return 1;
//     }
    
//     cyw43_arch_enable_sta_mode();

//     if (cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
//         printf("wifi connection failed\n");
//         return 1;
//     }
//     printf("IP address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));

//     tcp_server_t *server = tcp_server_open(); 
//     if (!server) {
//         printf("Failed to start TCP server\n");
//         while(1); // Halt
//     }

//     uint32_t tcp_timer = to_ms_since_boot(get_absolute_time());


//     float x = 0, y = 0, theta = 0;


//     while (true) {
//         Timer_Update(); // Met à jour les timers
//         int c;
//         cyw43_arch_poll();
//         // Toutes les secondes
//         // if ((Timer_ms1 - tcp_timer) > 1000) {
            
//         //     printf("client_pcb=%p, can_send=%d\n", server->client_pcb, server->can_send);  
//         //     // send_pose_to_foxglove(server, x, y, theta);

//         //     char msg[128];
//         //     snprintf(msg, sizeof(msg), "{\"x\":%.3f,\"y\":%.3f,\"theta\":%.3f}\n\0", x, y, theta);
//         //     tcp_server_send_data(server, (uint8_t*)msg, strlen(msg));

//         //     // simulation d’évolution
//         //     x += 0.05f;
//         //     theta += 0.01f;

//         //     tcp_timer = to_ms_since_boot(get_absolute_time());
//         // }
//         if ((Timer_ms1 - tcp_timer) > 1000) {

//             // printf("client_pcb=%p, is_handshake_complete=%d\n", server->client_pcb, server->complete);

//             char msg[128];
//             snprintf(msg, sizeof(msg), "{\"x\":%.3f,\"y\":%.3f,\"theta\":%.3f}\n\0", x, y, theta);
//             tcp_server_send_data(server, (uint8_t*)msg, strlen(msg));
            
//             // NEW: Call the Foxglove sender function
//             // send_pose_to_foxglove(server, x, y, theta);

//             // simulation d’évolution
//             x += 0.05f;
//             theta += 0.01f;

//             tcp_timer = to_ms_since_boot(get_absolute_time());
//         }

//         // Met à jour les moteurs pas à pas
//         Move_Loop();

//         switch (sequencer) {
//             case 0:
//                 c = getchar_timeout_us(0);
// 				if (c >= 0) {
// 					Interp(c);
// 				}
//                 sequencer++;
//                 break;
//             case 1:
//                 Asserv_Loop();
//                 sequencer++;
//                 break;

//             default:
//                 sequencer = 0;
//                 break;
//         }
//     }
//     cyw43_arch_deinit();
//     return 0;
// }

void Init_All(void)
{
    init_motors();
    Init_Asserv();
}


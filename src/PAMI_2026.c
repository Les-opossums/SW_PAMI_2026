#include "PAMI_2026.h"

int main()
{
    stdio_init_all();
    
    int sequencer = 0;

    Init_All();

    printf("PAMI-2026 ready.\n");

    char ssid[] = "Opossum";           // your network SSID (name)
    char password[] = "r28w3fr7j3zu8r4";       // your network password

    // Initialise la puce CYW43 (WiFi + Bluetooth)
    if (cyw43_arch_init_with_country(CYW43_COUNTRY_FRANCE)) {
        printf("failed to initialise\n");
        return 1;
    }
    
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("wifi connection failed\n");
        return 1;
    }
    printf("IP address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));

    tcp_server_t *server = tcp_server_open(); 
    if (!server) {
        printf("Failed to start TCP server\n");
        while(1); // Halt
    }

    uint32_t tcp_timer = to_ms_since_boot(get_absolute_time());


    float x = 0, y = 0, theta = 0;


    while (true) {
        Timer_Update(); // Met à jour les timers
        int c;
        cyw43_arch_poll();
        // Toutes les secondes
        // if ((Timer_ms1 - tcp_timer) > 1000) {
            
        //     printf("client_pcb=%p, can_send=%d\n", server->client_pcb, server->can_send);  
        //     // send_pose_to_foxglove(server, x, y, theta);

        //     char msg[128];
        //     snprintf(msg, sizeof(msg), "{\"x\":%.3f,\"y\":%.3f,\"theta\":%.3f}\n\0", x, y, theta);
        //     tcp_server_send_data(server, (uint8_t*)msg, strlen(msg));

        //     // simulation d’évolution
        //     x += 0.05f;
        //     theta += 0.01f;

        //     tcp_timer = to_ms_since_boot(get_absolute_time());
        // }
        if ((Timer_ms1 - tcp_timer) > 1000) {

            printf("client_pcb=%p, is_handshake_complete=%d\n", server->client_pcb, server->complete);

            // char msg[128];
            // snprintf(msg, sizeof(msg), "{\"x\":%.3f,\"y\":%.3f,\"theta\":%.3f}\n\0", x, y, theta);
            // tcp_server_send_data(server, (uint8_t*)msg, strlen(msg));
            
            // NEW: Call the Foxglove sender function
            send_pose_to_foxglove(server, x, y, theta);

            // simulation d’évolution
            x += 0.05f;
            theta += 0.01f;

            tcp_timer = to_ms_since_boot(get_absolute_time());
        }

        // Met à jour les moteurs pas à pas
        Move_Loop();

        switch (sequencer) {
            case 0:
                c = getchar_timeout_us(0);
				if (c >= 0) {
					Interp(c);
				}
                sequencer++;
                break;
            case 1:
                Asserv_Loop();
                sequencer++;
                break;

            default:
                sequencer = 0;
                break;
        }
    }
    cyw43_arch_deinit();
    return 0;
}

void Init_All(void)
{
    init_motors();
    Init_Asserv();
}


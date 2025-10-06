#include "PAMI_2026.h"

int main()
{
    stdio_init_all();
    
    int sequencer = 0;

    Init_All();

    printf("PAMI-2026 ready.\n");

    char ssid[] = "Opossum";           // your network SSID (name)
    char password[] = "xxxx";       // your network password

    if (cyw43_arch_init_with_country(CYW43_COUNTRY_FRANCE)) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("wifi connection failed\n");
        return 1;
    }

    TCP_SERVER_T *server = tcp_server_init();
    if (!server) {
        printf("failed to create server\n");
        return 1;
    }
    if (!tcp_server_open(server)) {
        printf("failed to open server\n");
        return 1;
    }

    uint32_t tcp_timer = to_ms_since_boot(get_absolute_time());


    float x = 0, y = 0, theta = 0;


    while (true) {
        Timer_Update(); // Met à jour les timers
        int c;
        cyw43_arch_poll();
        // Toutes les secondes
        if ((Timer_ms1 - tcp_timer) > 1000) {
            
                
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
}

void Init_All(void)
{
    init_motors();
    Init_Asserv();
}


#include "PAMI_2026.h"

int main()
{
    stdio_init_all();
    
    int sequencer = 0;

    Init_All();

    printf("PAMI-2026 ready.\n");

    char ssid[] = "Opossum";           // your network SSID (name)
    char password[] = "XXXXXXX";       // your network password

    if (cyw43_arch_init_with_country(CYW43_COUNTRY_FRANCE)) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("wifi connection failed\n");
        return 1;
    }

    TCP_CLIENT_T *state = (TCP_CLIENT_T *)calloc(1, sizeof(TCP_CLIENT_T));
    if (!state) {
        printf("failed to allocate state\n");
        return 1;
    }

    ip4addr_aton(TEST_TCP_SERVER_IP, &state->remote_addr);

    printf("Connecting to %s port %u\n", ip4addr_ntoa(&state->remote_addr), TCP_PORT);
    connect_client(state);

    uint32_t tcp_timer = to_ms_since_boot(get_absolute_time());

    while (true) {
        Timer_Update(); // Met à jour les timers
        int c;

        // Affichage des données reçues
        if (state->rx_buffer_len) {
            printf("%.*s\r\n", state->rx_buffer_len, (char *)state->rx_buffer);
            state->rx_buffer_len = 0;
        }

        // Toutes les secondes
        if ((Timer_ms1 - tcp_timer) > 1000) {
            if (state->connected == TCP_CONNECTED) {
                char test[20] = "test";
                state->buffer_len = WS_BuildPacket((char *)state->buffer, BUF_SIZE,
                                                   WEBSOCKET_OPCODE_TEXT, test,
                                                   strlen(test), 1);

                err_t err = tcp_write(state->tcp_pcb, state->buffer, state->buffer_len, TCP_WRITE_FLAG_COPY);
                if (err != ERR_OK) {
                    printf("Failed to write data %d\n", err);
                }
            }

            if (state->connected == TCP_DISCONNECTED) {
                connect_client(state);
            }

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
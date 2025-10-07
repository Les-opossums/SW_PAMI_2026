#include "PAMI_2026.h"

void send_pose_to_foxglove(tcp_server_t *state, float x, float y, float theta) {
    if (!state || !state->client_pcb) return;
    if (!state->complete) return;

    char json[128];
    snprintf(json, sizeof(json),
             "{\"x\":%.3f, \"y\":%.3f, \"theta\":%.3f}", x, y, theta);

    // Construire le paquet WebSocket (serveur → client : pas de mask)
    uint64_t len = WS_BuildPacket(
        (char *)state->buffer_sent,
        TCP_SERVER_BUF_SIZE,
        WEBSOCKET_OPCODE_TEXT,
        json,
        strlen(json),
        0 // pas de mask côté serveur !
    );

    // Envoyer le paquet via TCP
    cyw43_arch_lwip_check();
    err_t err = tcp_write(state->client_pcb, state->buffer_sent, len, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        printf("Failed to send to Foxglove: %d\n", err);
    } else {
        tcp_output(state->client_pcb);
        printf("Sent to Foxglove: %s\n", json);
    }
}

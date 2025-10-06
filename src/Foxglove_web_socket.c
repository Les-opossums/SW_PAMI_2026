#include "PAMI_2026.h"

void send_pose_to_foxglove(TCP_CLIENT_T *state, float x, float y, float theta) {
    if (!state) return;
    if (state->connected != TCP_CONNECTED) return;

    char json[128];
    snprintf(json, sizeof(json),
             "{\"x\":%.3f, \"y\":%.3f, \"theta\":%.3f}", x, y, theta);

    // Construire le paquet WebSocket
    uint64_t len = WS_BuildPacket(
        (char *)state->buffer,
        BUF_SIZE,
        WEBSOCKET_OPCODE_TEXT,
        json,
        strlen(json),
        1 // activer le mask (obligatoire côté client)
    );

    // Envoyer le paquet via TCP
    err_t err = tcp_write(state->tcp_pcb, state->buffer, len, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        printf("Failed to send: %d\n", err);
    } else {
        printf("Sent: %s\n", json);
    }
}


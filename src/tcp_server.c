#include "PAMI_2026.h"

err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, uint16_t len) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;
    printf("tcp_client_sent %u\n", len);
    return ERR_OK;
}

err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;
    if (err != ERR_OK) {
        printf("Connect failed %d\n", err);
        return err;
    }
    // Envoi de la requête HTTP pour upgrade WebSocket
    state->buffer_len = sprintf((char *)state->buffer,
        "GET / HTTP/1.1\r\n"
        "Host: REMOVED:8082\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: x3JJHMbDL1EzLkh9GBhXDw==\r\n"
        "Sec-WebSocket-Protocol: chat, superchat\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n");

    err = tcp_write(state->tcp_pcb, state->buffer, state->buffer_len, TCP_WRITE_FLAG_COPY);

    state->connected = TCP_CONNECTED;

    printf("Connected\r\n");
    return ERR_OK;
}

err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb) {
    printf("tcp_client_poll\n");
    return ERR_OK;
}

void tcp_client_err(void *arg, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;
    state->connected = TCP_DISCONNECTED;
    if (err != ERR_ABRT) {
        printf("tcp_client_err %d\n", err);
    } else {
        printf("tcp_client_err abort %d\n", err);
    }
}

err_t tcp_client_close(void *arg) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;
    err_t err = ERR_OK;

    if (state->tcp_pcb != NULL) {
        tcp_arg(state->tcp_pcb, NULL);
        tcp_poll(state->tcp_pcb, NULL, 0);
        tcp_sent(state->tcp_pcb, NULL);
        tcp_recv(state->tcp_pcb, NULL);
        tcp_err(state->tcp_pcb, NULL);
        err = tcp_close(state->tcp_pcb);
        if (err != ERR_OK) {
            printf("close failed %d, calling abort\n", err);
            tcp_abort(state->tcp_pcb);
            err = ERR_ABRT;
        }
        state->tcp_pcb = NULL;
    }

    state->connected = TCP_DISCONNECTED;
    return err;
}

err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;

    if (!p) {
        tcp_client_close(arg);
        return ERR_OK;
    }

    cyw43_arch_lwip_check();
    state->rx_buffer_len = 0;

    if (p->tot_len > 0) {
        for (struct pbuf *q = p; q != NULL; q = q->next) {
            if ((state->rx_buffer_len + q->len) < BUF_SIZE) {
                WebsocketPacketHeader_t header;
                WS_ParsePacket(&header, (char *)q->payload, q->len);
                memcpy(state->rx_buffer + state->rx_buffer_len,
                       (uint8_t *)q->payload + header.start,
                       header.length);
                state->rx_buffer_len += header.length;
            }
        }
        tcp_recved(tpcb, p->tot_len);
    }

    pbuf_free(p);
    return ERR_OK;
}

err_t connect_client(void *arg) {
    TCP_CLIENT_T *state = (TCP_CLIENT_T *)arg;

    if (state->connected != TCP_DISCONNECTED) return ERR_OK;

    state->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&state->remote_addr));
    if (!state->tcp_pcb) {
        printf("failed to create tcp pcb\n");
        return ERR_MEM;
    }

    tcp_arg(state->tcp_pcb, state);
    tcp_poll(state->tcp_pcb, tcp_client_poll, 1);
    tcp_sent(state->tcp_pcb, tcp_client_sent);
    tcp_recv(state->tcp_pcb, tcp_client_recv);
    tcp_err(state->tcp_pcb, tcp_client_err);

    state->buffer_len = 0;

    cyw43_arch_lwip_begin();
    state->connected = TCP_CONNECTING;
    err_t err = tcp_connect(state->tcp_pcb, &state->remote_addr, TCP_PORT, tcp_client_connected);
    cyw43_arch_lwip_end();

    return err;
}

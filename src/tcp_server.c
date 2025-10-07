#include "PAMI_2026.h"

tcp_server_t *tcp_server_init(void) {
    tcp_server_t *state = calloc(1, sizeof(tcp_server_t));
    if (!state) {
        printf("Failed to allocate tcp_server_t\n");
        return NULL;
    }
    return state;
}

err_t tcp_server_close(void *arg) {
    tcp_server_t *state = (tcp_server_t *)arg;
    err_t err = ERR_OK;

    if (state->client_pcb) {
        tcp_arg(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_sent(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        err = tcp_close(state->client_pcb);
        if (err != ERR_OK) {
            printf("TCP close failed (%d), aborting\n", err);
            tcp_abort(state->client_pcb);
            err = ERR_ABRT;
        }
        state->client_pcb = NULL;
    }

    if (state->server_pcb) {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }

    return err;
}

err_t tcp_server_result(void *arg, int status) {
    tcp_server_t *state = (tcp_server_t *)arg;
    if (status == 0)
        printf("TCP test success\n");
    else
        printf("TCP test failed (%d)\n", status);

    state->connected = true;
    return tcp_server_close(arg);
}

err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    tcp_server_t *state = (tcp_server_t *)arg;
    state->sent_len += len;

    if (state->sent_len >= TCP_SERVER_BUF_SIZE) {
        state->recv_len = 0;
        printf("Waiting for buffer from client\n");
    }

    return ERR_OK;
}

err_t tcp_server_send_data(void *arg, struct tcp_pcb *tpcb) {
    tcp_server_t *state = (tcp_server_t *)arg;
    for (int i = 0; i < TCP_SERVER_BUF_SIZE; i++) {
        state->buffer_sent[i] = rand();
    }

    state->sent_len = 0;
    printf("Writing %d bytes to client\n", TCP_SERVER_BUF_SIZE);
    cyw43_arch_lwip_check();

    err_t err = tcp_write(tpcb, state->buffer_sent, TCP_SERVER_BUF_SIZE, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        printf("tcp_write failed: %d\n", err);
        return tcp_server_result(arg, -1);
    }

    return ERR_OK;
}


err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    tcp_server_t *state = (tcp_server_t *)arg;

    if (!p) {
        return tcp_server_result(arg, -1);
    }

    cyw43_arch_lwip_check();

    if (p->tot_len > 0) {
        const uint16_t buffer_left = TCP_SERVER_BUF_SIZE - state->recv_len;
        state->recv_len += pbuf_copy_partial(p,
                                             state->buffer_recv + state->recv_len,
                                             p->tot_len > buffer_left ? buffer_left : p->tot_len,
                                             0);
        tcp_recved(tpcb, p->tot_len);
    }

    pbuf_free(p);

    if (state->recv_len == TCP_SERVER_BUF_SIZE) {
        if (memcmp(state->buffer_sent, state->buffer_recv, TCP_SERVER_BUF_SIZE) != 0) {
            printf("Buffer mismatch\n");
            return tcp_server_result(arg, -1);
        }

        printf("Buffer verified OK\n");
        state->run_count++;

        if (state->run_count >= TCP_SERVER_TEST_ITERATIONS) {
            tcp_server_result(arg, 0);
            return ERR_OK;
        }

        return tcp_server_send_data(arg, state->client_pcb);
    }

    return ERR_OK;
}

err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) {
    printf("TCP server poll timeout\n");
    return tcp_server_result(arg, -1);
}

void tcp_server_err(void *arg, err_t err) {
    if (err != ERR_ABRT) {
        printf("TCP server error: %d\n", err);
        tcp_server_result(arg, err);
    }
}


err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    tcp_server_t *state = (tcp_server_t *)arg;

    if (err != ERR_OK || client_pcb == NULL) {
        printf("Accept error: %d\n", err);
        return tcp_server_result(arg, err);
    }

    printf("Client connected\n");
    state->client_pcb = client_pcb;

    tcp_arg(client_pcb, state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, TCP_SERVER_POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    return tcp_server_send_data(arg, client_pcb);
}


bool tcp_server_open(void *arg) {
    tcp_server_t *state = tcp_server_init();
    if (!state) return false;

    printf("Starting TCP server on port %u\n", TCP_SERVER_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        printf("Failed to create PCB\n");
        free(state);
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_SERVER_PORT);
    if (err != ERR_OK) {
        printf("Failed to bind to port %u\n", TCP_SERVER_PORT);
        tcp_close(pcb);
        free(state);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        printf("Failed to listen\n");
        tcp_close(pcb);
        free(state);
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);
    return true;
}



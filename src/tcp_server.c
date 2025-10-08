#include "PAMI_2026.h"

tcp_server_t *tcp_server_init(void) {
    tcp_server_t *state = calloc(1, sizeof(tcp_server_t));
    state->can_send = false;
    state->is_connected = false;
    if (!state) {
        DEBUG_printf("Failed to allocate tcp_server_t\n");
        return NULL;
    }
    return state;
}

err_t tcp_server_close(void *arg) {
    DEBUG_printf("Closing TCP server\n");
    tcp_server_t *state = (tcp_server_t *)arg;
    err_t err = ERR_OK;

    if (state->client_pcb != NULL) {
        printf("Closing client connection. Setting is_connected = false\n");
        tcp_arg(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_sent(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        state->can_send = false;
        state->is_connected = false;
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

err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    tcp_server_t *state = (tcp_server_t *)arg;
    state->can_send = true;
    return ERR_OK;
}

err_t tcp_server_send_data(tcp_server_t *state, const uint8_t *data, size_t len) {
    if (!state || !state->client_pcb || !state->is_connected) {
        printf("[Sender] No client connected\n");
        return ERR_VAL;
    }

    // --- FIX: Check if we are allowed to send data ---
    if (!state->can_send) {
        printf("[Sender] Cannot send now, waiting for previous send to complete.\n");
        // Return a non-fatal error; we can try again later.
        return ERR_INPROGRESS; 
    }

    if (len == 0) {
        printf("[Sender] Invalid length 0\n");
        return ERR_VAL;
    }
    
    // Check if the send buffer has enough space.
    // This is a more robust check than just a flag.
    if (tcp_sndbuf(state->client_pcb) < len) {
        printf("[Sender] Not enough space in send buffer. Available: %u, Required: %zu\n",
               tcp_sndbuf(state->client_pcb), len);
        return ERR_MEM;
    }

    // The memcpy to state->buffer_sent is not necessary because you are using
    // TCP_WRITE_FLAG_COPY, which tells lwIP to make its own copy.
    // I've removed it for simplicity, but your original way also works.
    
    cyw43_arch_lwip_begin();
    err_t err = tcp_write(state->client_pcb, data, len, TCP_WRITE_FLAG_COPY);
    
    if (err == ERR_OK) {
        // --- FIX: Immediately prevent further sends until this one is acknowledged ---
        state->can_send = false;
        err = tcp_output(state->client_pcb);
    }
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("[Sender] Failed to write data: %d\n", err);
    }

    return err;
}



err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    tcp_server_t *state = (tcp_server_t *)arg;

    if (!p) { // Connection closed by client
        printf("Client DISCONNECTED. Setting is_connected = false\n");
        state->is_connected = false;
        state->client_pcb = NULL;
        tcp_close(tpcb);
        printf("Client disconnected\n");
        // pbuf_free(p);
        return ERR_OK;
    }

    cyw43_arch_lwip_check();

    if (p->tot_len > 0) {
        const uint16_t buffer_left = TCP_SERVER_BUF_SIZE - state->recv_len;
        state->recv_len = pbuf_copy_partial(p, state->buffer_recv, sizeof(state->buffer_recv) - 1, 0);
        state->buffer_recv[state->recv_len] = '\0'; // Null-terminate for printing
        tcp_recved(tpcb, p->tot_len);
    }

    // traitement des données reçues print dans le terminal
    DEBUG_printf("Received %d bytes from client\n", p->tot_len);
    DEBUG_printf("Data: ");
    for (int i = 0; i < p->tot_len; i++) {
        char c = state->buffer_recv[i];
        // On remplace les caractères non imprimables par un point
        if (c >= 32 && c <= 126) {
            DEBUG_printf("%c", c);
        } else {
            DEBUG_printf(".");
        }
    }
    DEBUG_printf("\n");

    pbuf_free(p);

    return ERR_OK;
}

err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) { 
    tcp_server_t *state = (tcp_server_t *)arg;
    return ERR_OK;
}

void tcp_server_err(void *arg, err_t err) {
    if (err != ERR_ABRT) {
        printf("TCP server error: %d\n", err);
    } else {
        printf("TCP server aborted\n");
    }
}


err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    tcp_server_t *state = (tcp_server_t *)arg;

    if (err != ERR_OK || client_pcb == NULL) {
        printf("Accept error: %d\n", err);
        return ERR_VAL;
    }

    printf("Client connected ACCEPTED. Setting is_connected = true\n");
    state->client_pcb = client_pcb;
    state->is_connected = true;
    state->can_send = true;

    tcp_arg(client_pcb, state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, TCP_SERVER_POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    return ERR_OK;
}


tcp_server_t* tcp_server_open(void) {
    tcp_server_t *state = tcp_server_init();
    if (!state) return NULL;

    printf("Starting TCP server on port %u\n", TCP_SERVER_PORT);

    // STEP 1: Create the PCB
    printf("Attempting to create new PCB...\n");
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        printf("--> FAILED: tcp_new_ip_type returned NULL. Out of PCBs!\n");
        free(state);
        return NULL;
    }
    printf("PCB created successfully.\n");

    // STEP 2: Bind to the port
    printf("Attempting to bind to port %u...\n", TCP_SERVER_PORT);
    err_t err = tcp_bind(pcb, NULL, TCP_SERVER_PORT);
    if (err != ERR_OK) {
        printf("--> FAILED: tcp_bind returned error %d\n", err);
        tcp_close(pcb);
        free(state);
        return NULL;
    }
    printf("Bind successful.\n");

    // STEP 3: Listen for connections
    printf("Attempting to listen...\n");
    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        printf("--> FAILED: tcp_listen_with_backlog returned NULL\n");
        tcp_close(pcb);
        free(state);
        return NULL;
    }
    printf("Server is now listening.\n");

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);
    return state;
}


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

    if (!p) { // Connection closed
        printf("Client disconnected.\n");
        state->is_connected = false;
        state->client_pcb = NULL;
        tcp_close(tpcb);
        return ERR_OK;
    }

    cyw43_arch_lwip_check();

    if (p->tot_len > 0) {
        // Limitation de sécurité pour éviter le Buffer Overflow
        uint16_t len = p->tot_len;
        if (len > sizeof(state->buffer_recv) - 1) {
            len = sizeof(state->buffer_recv) - 1;
        }

        // Copie les données reçues
        pbuf_copy_partial(p, state->buffer_recv, len, 0);
        state->buffer_recv[len] = '\0';
        
        // --- INJECTION DANS L'INTERPRÉTEUR ---
        for(uint16_t i = 0; i < len; i++) {
            Interp((int)state->buffer_recv[i]);
        }

        tcp_recved(tpcb, p->tot_len); // On acquitte toujours la taille totale à lwIP
    }

    pbuf_free(p);
    return ERR_OK;
}

err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) { 
    tcp_server_t *state = (tcp_server_t *)arg;
    return ERR_OK;
}

void tcp_server_err(void *arg, err_t err) {
    tcp_server_t *state = (tcp_server_t *)arg;
    if (err != ERR_ABRT) {
        printf("TCP server error: %d\n", err);
    } else {
        printf("TCP server aborted\n");
    }

    if (state != NULL) {
        state->client_pcb = NULL;
        state->is_connected = false;
        state->can_send = false;
    }
}


err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    tcp_server_t *state = (tcp_server_t *)arg;

    if (err != ERR_OK || client_pcb == NULL) {
        printf("Accept error: %d\n", err);
        return ERR_VAL;
    }

    // On informe lwIP que la connexion est gérée, sinon il refusera 
    // systématiquement les futures connexions !
    tcp_accepted(state->server_pcb);

    // --- Tuer l'ancienne connexion fantôme s'il y en a une ---
    if (state->client_pcb != NULL) {
        printf("Un client est deja connecte. Fermeture de l'ancienne connexion.\n");
        tcp_abort(state->client_pcb); // On tue l'ancienne
        state->client_pcb = NULL;
    }

    printf("Client connected ACCEPTED. Setting is_connected = true\n");
    state->client_pcb = client_pcb;
    // ... (le reste de ta fonction reste inchangé)
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

    cyw43_arch_lwip_begin(); 

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        printf("--> FAILED: tcp_new_ip_type returned NULL. Out of PCBs!\n");
        cyw43_arch_lwip_end();
        free(state);
        return NULL;
    }

    err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_SERVER_PORT);
    if (err != ERR_OK) {
        printf("--> FAILED: tcp_bind returned error %d\n", err);
        tcp_close(pcb);
        cyw43_arch_lwip_end();
        free(state);
        return NULL;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        printf("--> FAILED: tcp_listen_with_backlog returned NULL\n");
        tcp_close(pcb);
        cyw43_arch_lwip_end();
        free(state);
        return NULL;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);
    
    // --- Fin de la zone critique lwIP ---
    cyw43_arch_lwip_end(); 
    
    printf("Server is now listening.\n");
    return state;
}


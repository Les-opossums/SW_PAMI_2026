#include "PAMI_2026.h"

tcp_server_t *tcp_server_init(void) {
    tcp_server_t *state = calloc(1, sizeof(tcp_server_t));
    if (!state) {
        DEBUG_printf("Failed to allocate tcp_server_t\n");
        return NULL;
    }
    state->can_send = false;
    state->is_connected = false;
    state->is_websocket_ready = false; // Initialisation
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
        state->is_websocket_ready = false; // Reset
        
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

    // --- Vérifie si on a le droit d'envoyer ---
    if (!state->can_send) {
        printf("[Sender] Cannot send now, waiting for previous send to complete.\n");
        return ERR_INPROGRESS; 
    }

    if (len == 0) {
        printf("[Sender] Invalid length 0\n");
        return ERR_VAL;
    }
    
    // Vérifier si le buffer lwIP a assez de place
    if (tcp_sndbuf(state->client_pcb) < len) {
        printf("[Sender] Not enough space in send buffer. Available: %u, Required: %zu\n",
               tcp_sndbuf(state->client_pcb), len);
        return ERR_MEM;
    }

    cyw43_arch_lwip_begin();
    err_t err = tcp_write(state->client_pcb, data, len, 0); // On met 0 car on gère notre propre mémoire static
    
    if (err == ERR_OK) {
        state->can_send = false; // Bloque les prochains envois jusqu'à l'ACK
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

    if (!p) { // Connexion fermée par le client
        printf("Client disconnected.\n");
        state->is_connected = false;
        state->is_websocket_ready = false; // Reset au débranchement
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

        // Copie des données reçues dans le buffer local
        pbuf_copy_partial(p, state->buffer_recv, len, 0);
        state->buffer_recv[len] = '\0';
        
        // --- ÉTAPE 1 : POIGNÉE DE MAIN HTTP (HANDSHAKE WEBSOCKET) ---
        if (!state->is_websocket_ready) {
            // On cherche la clé envoyée par le navigateur
            char *key_start = strstr((char*)state->buffer_recv, "Sec-WebSocket-Key: ");
            if (key_start) {
                key_start += 19; // On avance jusqu'à la valeur de la clé
                char *key_end = strchr(key_start, '\r');
                if (key_end) {
                    *key_end = '\0'; // On isole la clé
                    
                    // On concatène avec le "Magic String" officiel de la RFC 6455
                    char concat_key[100];
                    snprintf(concat_key, sizeof(concat_key), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", key_start);

                    // Hachage SHA-1 avec mbedtls
                    unsigned char sha1_sum[20];
                    mbedtls_sha1((unsigned char*)concat_key, strlen(concat_key), sha1_sum);
                    // Encodage Base64 avec mbedtls
                    char base64_key[64];
                    size_t base64_len;
                    mbedtls_base64_encode((unsigned char*)base64_key, sizeof(base64_key), &base64_len, sha1_sum, 20);

                    // Création de la réponse HTTP 101 pour valider le WebSocket
                    char response[256];
                    snprintf(response, sizeof(response),
                        "HTTP/1.1 101 Switching Protocols\r\n"
                        "Upgrade: websocket\r\n"
                        "Connection: Upgrade\r\n"
                        "Sec-WebSocket-Accept: %s\r\n\r\n", base64_key);

                    // Envoi immédiat de la réponse HTTP
                    tcp_write(tpcb, response, strlen(response), TCP_WRITE_FLAG_COPY);
                    tcp_output(tpcb);
                    
                    state->is_websocket_ready = true;
                    printf("WebSocket Handshake OK !\n");
                }
            }
        } 
        // --- ÉTAPE 2 : PARSING DES COMMANDES WEBSOCKET ---
        else {
            WebsocketPacketHeader_t ws_header;
            if (WS_ParsePacket(&ws_header, (char*)state->buffer_recv, len) == 0) {
                
                // Si c'est une trame texte (OPCODE 1)
                if (ws_header.meta.bits.OPCODE == WEBSOCKET_OPCODE_TEXT) {
                    char *payload = (char*)state->buffer_recv + ws_header.start;
                    
                    // --- DEBUG : Afficher la trame texte reçue (les 6 premiers caractères) ---
                    printf("[C DEBUG] Trame texte reçue: '%.*s'\n", ws_header.length > 6 ? 6 : ws_header.length, payload);

                    if (strncmp(payload, "WHOAMI", 6) == 0) {
                        char response[32];
                        snprintf(response, sizeof(response), "PAMI_ID:%d", ROBOT_ID);
                        
                        printf("[C DEBUG] Commande WHOAMI reconnue ! Envoi de : %s\n", response);
                        
                        char ws_buf[128];
                        uint64_t pack_len = WS_BuildPacket(ws_buf, sizeof(ws_buf), 
                                                           WEBSOCKET_OPCODE_TEXT, 
                                                           response, strlen(response), 0);
                        tcp_server_send_data(state, (uint8_t*)ws_buf, pack_len);
                    } else {
                        printf("[C DEBUG] Commande non reconnue reçue: '%.*s'\n", ws_header.length > 6 ? 6 : ws_header.length, payload);
                        for(uint16_t i = 0; i < ws_header.length; i++) {
                            Interp((int)payload[i]);
                        }
                    }
                }
                // Si la trame est une demande de déconnexion (OPCODE 8)
                else if (ws_header.meta.bits.OPCODE == WEBSOCKET_OPCODE_CLOSE) {
                    printf("WebSocket Client requested closure. Fermeture propre.\n");
                    
                    // On libère la mémoire lwIP avant de tuer la connexion
                    tcp_recved(tpcb, p->tot_len);
                    pbuf_free(p);
                    
                    // On reset l'état
                    state->is_connected = false;
                    state->is_websocket_ready = false;
                    state->client_pcb = NULL;
                    
                    // Fermeture douce du socket
                    tcp_close(tpcb);
                    return ERR_OK;
                }
            }
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
        state->is_websocket_ready = false;
        state->can_send = false;
    }
}


err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    tcp_server_t *state = (tcp_server_t *)arg;

    if (err != ERR_OK || client_pcb == NULL) {
        printf("Accept error: %d\n", err);
        return ERR_VAL;
    }

    // ---> LA LIGNE MAGIQUE POUR AUTORISER LES FUTURES RECONNEXIONS <---
    tcp_accepted(state->server_pcb);

    // Si un ancien client fantôme est là, on le dégage
    if (state->client_pcb != NULL) {
        tcp_abort(state->client_pcb);
    }

    printf("Client connected ACCEPTED. Setting is_connected = true\n");
    state->client_pcb = client_pcb;
    state->is_connected = true;
    state->can_send = true;
    state->is_websocket_ready = false; // Nouveau client = doit faire le Handshake !

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
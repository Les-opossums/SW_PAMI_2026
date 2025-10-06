#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#define TEST_TCP_SERVER_IP "10.229.162.XXX"
#define TCP_PORT 8082

#define BUF_SIZE 2048

#define TCP_DISCONNECTED 0
#define TCP_CONNECTING   1
#define TCP_CONNECTED    2

typedef struct {
    struct tcp_pcb *tcp_pcb;
    ip_addr_t remote_addr;
    uint8_t buffer[BUF_SIZE];
    uint8_t rx_buffer[BUF_SIZE];
    int buffer_len;
    int rx_buffer_len;
    int sent_len;
    int connected;
} TCP_CLIENT_T;

err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, uint16_t len);
err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb);

void tcp_client_err(void *arg, err_t err);
err_t tcp_client_close(void *arg);
err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
err_t connect_client(void *arg);

#endif // TCP_SERVER_H
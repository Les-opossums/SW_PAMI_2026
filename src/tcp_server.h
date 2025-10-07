#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#define TCP_SERVER_PORT 4242
#define TCP_SERVER_BUF_SIZE 2048
#define TCP_SERVER_TEST_ITERATIONS 10
#define TCP_SERVER_POLL_TIME_S 5

typedef struct {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    bool connected;

    uint8_t buffer_sent[TCP_SERVER_BUF_SIZE];
    uint8_t buffer_recv[TCP_SERVER_BUF_SIZE];

    int sent_len;
    int recv_len;
    int run_count;
} tcp_server_t;

tcp_server_t* tcp_server_init(void);
err_t tcp_server_close(void *arg);
err_t tcp_server_result(void *arg, int status);
err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
err_t tcp_server_send_data(void *arg, struct tcp_pcb *tpcb);
err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);
void tcp_server_err(void *arg, err_t err);
err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err);
bool tcp_server_open(void *arg);

#endif // TCP_SERVER_H
#include "PAMI_2026.h"

int main()
{
    stdio_init_all();

    LIDAR_UART_init();
    
    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}

